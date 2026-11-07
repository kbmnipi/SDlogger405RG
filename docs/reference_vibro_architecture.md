# VIBRO-ANALYZER — Production Architecture Strategy
**Target:** STM32F407VG (Discovery) · LIS3DSH @1600 Hz · ST7735 · CMSIS-DSP FFT-1024

---

## 1. Executive Summary (Prototype vs Production)

The prototype is a **single-threaded, flag-driven super-loop**: a TIM2 ISR at 1600 Hz bit-bangs two SPI register reads per sample into a float buffer; when 1024 samples accumulate, `loop()` *pauses the timer*, runs DC-removal + Hann windowing + `arm_rfft_fast_f32`, and redraws the ST7735 — then resumes sampling.

**Verdict:** functionally correct for demo purposes, but **not production-grade**. The three structural defects:

1. **Sampling is not continuous.** Timer pause/resume around each FFT+UI cycle creates 10–60 ms data gaps every ~640 ms (effective duty cycle < 97%). Transient vibration events in the gap are invisible; spectral estimates are biased by discontinuity.
2. **The ISR does real work** (~35–60 µs of blocking SPI per tick, plus `SPI.beginTransaction` overhead). Any higher-priority interrupt or display activity overlapping a tick corrupts timing and can tear SPI transactions (CS toggling is not atomic w.r.t. the display path — only safe today because sampling is paused while drawing).
3. **No fault containment:** no watchdog, no sensor self-test, no error logging, no CRC; a hung display write or lost DRDY flag silently degrades the device forever.

**Recommended target architecture:** **STM32 HAL + FreeRTOS**, DMA-driven acquisition into a lock-free ring buffer in DTCM, CMSIS-DSP FFT confined to CCM RAM, chunked SPI arbitration between sensor and display, IWDG + event log for long-term stability. CMSIS-DSP is framework-agnostic C — it survives the migration untouched; only the Arduino core and Adafruit GFX are replaced.

---

## 2. Framework & RTOS Evaluation

### 2.1 Is Arduino acceptable at 1600 Hz?
No, not for production. Not because of raw speed (the M4F @168 MHz is overkill), but because of **determinism and coupling**:

| Criterion | Arduino core | HAL + FreeRTOS |
|---|---|---|
| ISR latency bound | Unbounded by design; `SPI.transfer` in ISR blocks on peripheral flags with no preemption guarantee | Bounded: ISR only kicks DMA / appends to ring buffer (<2 µs CPU) |
| Memory placement | Implicit (BSS → D1 SRAM); cannot force CCM for DSP buffers without linker tricks | Explicit via scatter file / section attributes; deterministic map |
| Library coupling | Adafruit GFX pulls in Arduino `Stream`, `Print`, string heap — ~4–8 KB RAM + flash, hidden blocking (`delay()`) | Thin ST7735 driver over HAL SPI+DMA (~2 KB); no dynamic allocation at runtime |
| Concurrency model | One implicit task + ISRs; flags like `data_ready` are hand-rolled and fragile | Explicit producer/consumer tasks with SPSC queues; priorities documented |
| Watchdog / error policy | IWDG possible but awkward inside Arduino loop; no structured state machine | First-class: watchdog fed by a dedicated task, formal FSM |
| Maintainability | Fast to write, hard to certify; behavior depends on library internals | Slower to build, auditable; each layer has one job |

**Trade-off summary:** FreeRTOS adds ~10–20 KB RAM (kernel + stacks) and ~8–15 KB flash — negligible against the F407's 192 KB DTCM / 64 KB CCM / 1 MB flash. The latency win is structural: sampling jitter drops from "whatever `loop()` was doing" to a bounded worst case (<2 µs), because the acquisition path no longer shares CPU time with rendering.

### 2.2 Decision
- **Phase 1 (interim):** bare-metal HAL super-loop — proves the DMA/DSP pipeline without RTOS complexity.
- **Phase 2 (target):** **FreeRTOS** on top of HAL. Justification: three genuinely concurrent concerns (acquisition, DSP, UI) plus a watchdog/logging duty; hand-rolled flags are exactly what failed in the prototype. Tickless mode is *not* needed at this load — keep the kernel tick for simplicity and use it as the liveness signal.

---

## 3. Concurrency & Data Acquisition Strategy (1600 Hz)

### 3.1 Defects of the current ISR model
- Timer period (625 µs, HSE-derived) is **not synchronized to the LIS3DSH's internal clock**. ODR=1600 Hz means samples appear on DRDY at the sensor's own cadence; reading on TIM2 ticks causes phase drift and occasional double/missed reads.
- `data_ready` + pause/resume couples DSP latency directly into the sampling timeline (see §1).
- X-axis only: fine if the mounting axis is fixed, but note 3-axis would triple SPI traffic (12 bytes/sample) — plan headroom now.

### 3.2 Target model: DRDY-gated DMA producer → SPSC ring buffer → consumer tasks

**Producer (acquisition):**
1. TIM2 free-runs at **4× oversampling rate (6.4 kHz)** as a *polling clock*, not the sample clock.
2. Each tick, a short ISR reads `CTRL_REG3` and checks the **DRDY flag**. When set:
   - start an SPI RX DMA transfer of one sample (X_L/X_H; 4 SPI bytes with preloaded TX pattern — verify burst-read behavior against the LIS3DSH datasheet; if no burst, two back-to-back 2-byte transfers),
   - on DMA-complete callback, append the `int16` to a **power-of-two SPSC ring buffer** (e.g., 4096 samples = 8 KB) in DTCM. Head/tail indices are single-writer/single-reader → no locks needed; ISR uses only atomic index increments.
3. DRDY gating makes the sample timestamp *the sensor's own clock* — zero drift, and a missed tick self-corrects on the next DRDY.

**Consumer (DSP task):**
- Pops 1024-sample frames from the ring buffer (hop = 512 → ~6 fps spectrum refresh instead of the prototype's ~1.5 fps; hop is tunable).
- Converts to float, applies **precomputed Hann window table** (see §4), runs `arm_rfft_fast_f32`, computes magnitudes + peak bin.
- Publishes the spectrum into a double-buffered "latest frame" slot for the UI task.

**Consumer (UI task):**
- Renders chunked frames (§5); never touches the ring buffer directly.

### 3.3 Jitter budget (worst case)
| Event | Budget |
|---|---|
| TIM2 ISR latency (preempted by nothing higher-priority; DMA callbacks are µs-scale) | < 2 µs |
| SPI sample transfer @1 MHz, 4 bytes | ~35–40 µs (≈7% of period — acceptable); @10 MHz → ~4 µs |
| DRDY poll read per tick | +16 µs at 1 MHz; amortized over 4 ticks |
| DSP frame (rfft-1024 f32, FPU) | < 150 µs — fully off the sampling path |

**Result:** sample-to-sample period = sensor ODR ± a few µs of ISR jitter, with **no gaps**, because rendering and FFT never block acquisition. This is the core architectural win over the prototype.

### 3.4 Task table
| Task | Priority | Stack (est.) | Duty | Notes |
|---|---|---|---|---|
| Acquisition (ISR + DMA cb) | ISR / highest | — | ~10% | Only ring-buffer writes |
| DSP | High | 2 KB | <5% | FFT in CCM; publishes frame slot |
| UI | Normal | 4 KB | ~30–60% during refresh | Chunked SPI writes, feeds IWDG |
| Sys (watchdog/log/self-test) | Low | 1.5 KB | idle | Event ring buffer, sensor health |

---

## 4. Memory Map & DSP Integration (D1 vs CCM)

F407 memory: **DTCM/D1 SRAM = 192 KB @0x20000000** (CPU + DMA), **CCM RAM = 64 KB @0x10000000** (CPU-only — *DMA cannot reach it*).

### 4.1 Placement rules
| Object | Size | Location | Rationale |
|---|---|---|---|
| SPI RX DMA buffer(s) | ≤ 256 B | **D1 SRAM** | Hard requirement: DMA master can't address CCM |
| SPSC sample ring (int16, 4096) | 8 KB | **D1 SRAM** | Written by ISR/DMA callback; read by DSP task — both CPU-visible, D1 is the only safe shared home |
| FFT input / output / scratch | 4 + 8 + 2 = 14 KB | **CCM RAM** | Pure-CPU data; frees D1 for DMA and future buffers (e.g., 3-axis) |
| Hann window table (float, 1024) | 4 KB | **Flash (const)** or CCM | Precomputed once at init — the prototype recomputes 1024 `arm_cos_f32` per frame (~ms of waste); a const table in flash is free and deterministic |
| CMSIS twiddle tables | ~8–16 KB | Flash (library default) | Read-only; keep out of RAM entirely |
| FreeRTOS TCBs + kernel heap | ~4–8 KB | CCM or D1 | CPU-only access → CCM preferred if D1 is tight |
| Task stacks | 7.5 KB total | D1 SRAM | Keep DMA-adjacent memory predictable; either works, pick one and document it |

### 4.2 CMSIS-DSP integration notes
- `arm_rfft_fast_f32` (N=1024) is the right primitive: real-input FFT, FPU-optimized, ~10k MACs → well under 150 µs at 168 MHz. Keep it; it compiles identically under HAL/FreeRTOS with `-mfpu=fpv4-sp-d16 -mfloat-abi=hard`.
- **Latent bug in the prototype:** `arm_rfft_fast_init_f32(&fft_handler, FFT_SIZE)` is called without explicitly allocating the instance's scratch buffer (`pScratch`, N/2 floats ≈ 2 KB). Depending on the CMSIS-DSP revision this is undefined behavior. In production: allocate scratch in CCM and assign it to the instance at init; add a unit test that runs the FFT on a known sine and checks peak bin/magnitude.
- Bin resolution = fs/N = 1600/1024 ≈ **1.56 Hz**, Nyquist 800 Hz (512 usable bins) — matches the prototype's UI math; keep it. If finer resolution is ever needed, prefer N=2048 (still <300 µs) over zero-padding.
- Peak search: use squared magnitude (`re²+im²`) for comparison and take `sqrt` only once for display — removes 511 `sqrtf` calls per frame.

---

## 5. SPI Bus Arbitration (LIS3DSH + ST7735 on SPI1)

Both devices share SPI1 (SCK/MOSI/MISO), with CS: LIS=PE3, TFT=PE4. The prototype is only safe by accident (sampling paused while drawing). Production policy:

### 5.1 Chosen mechanism: **DMA-priority acquisition + chunked display writes under a bus mutex**
1. **Sensor path owns the fast lane.** Sample reads are DMA transfers of ≤4 bytes (~35 µs @1 MHz). The ISR/DMA callback only needs CS discipline for ~µs of CPU. It runs at higher priority and is *never blocked by UI work*.
2. **Display path is chunked.** A full ST7735 frame (fillRect 160×53 + bars ≈ 8–9 k pixels, RGB565) takes 20–40 ms if written contiguously — far too long to hold the bus. Split rendering into **row/bar chunks of ≤ ~100 µs SPI time each**; between chunks the UI task releases the SPI mutex (or simply yields), letting any pending sensor transfer complete.
3. **Arbitration object:** one `spi_bus` semaphore taken by the UI task per chunk; the acquisition path uses a short critical section (`portDISABLE_INTERRUPTS`) only for CS toggle + DMA start, never a blocking call. Invariant: *no SPI transaction from either device is ever longer than 100 µs*, so worst-case cross-device latency is bounded and data loss is impossible by construction.
4. **Per-device settings preserved:** sensor MODE3 @1–10 MHz; ST7735 @ up to ~10 MHz (verify against its datasheet max). Reconfigure via HAL `SPI_Init` per transaction or keep two preconfigured parameter sets.

### 5.2 Alternatives considered
- **Pure mutex everywhere** — rejected: ISRs can't take FreeRTOS semaphores; would force acquisition into a task and add scheduling jitter to the sample path.
- **DMA priority only, no chunking** — rejected: a 30 ms display DMA transfer would starve sensor reads for 30 ms → exactly the gap problem we're eliminating.
- **Hardware split (recommended if rework is allowed):** move the ST7735 to SPI2 (e.g., PA15/PC7/PB13/PB14) or use the board's second SPI. This eliminates arbitration entirely and is the cleanest long-term fix; treat it as a hardware option, not a software dependency — the chunking policy above must work regardless.

### 5.3 Sequence (textual)
```
UI task:            [take mutex] chunk0 → release → chunk1 → release ...
Acq ISR/DMA:        tick→DRDY? ──no──▶ return
                         │yes
                         ▼
                    CS_L, start DMA(4B), CS_H   (≤ µs CPU)
                    DMA cb: ring[tail++] = sample
Invariant: max transaction ≤ 100 µs ⇒ no starvation, no tearing.
```

---

## 6. System State Machine & Error Handling

### 6.1 Top-level FSM
```
        ┌──────────┐   WHO_AM_I OK    ┌───────────────┐
 BOOT ─▶│ INIT     │─────────────────▶│ ACQUIRE       │◀─────────────┐
        │ (SPI,TFT,│                  │ (steady state)│              │
        │ LIS init)│                  └──────┬───────┘              │
        └────┬─────┘                          │                     │
             │ ID mismatch / 3 retries fail   │ DRDY timeout / SPI err│
             ▼                               ▼                       │
      ┌────────────┐                 ┌──────────────┐               │
      │ SENSOR_FAULT│                │ RECOVER       │───────────────┘
      │ (safe UI)   │                │ (re-init LIS, │  (≤3 attempts)
      └────────────┘                │  flush ring)  │
                                    └──────┬───────┘
                                           │ display DMA timeout / 2nd fault
                                           ▼
                                    ┌──────────────┐   IWDG expiry
                                    │ DISPLAY_FAULT│────────▶ HW RESET
                                    └──────────────┘
```
- **ACQUIRE** is the only state where sampling runs; all other states render a static diagnostic frame and stop the ring buffer (or keep it running with data discarded — choose per product risk policy).
- Transitions are event-driven from the Sys task: DRDY counter, SPI error flags, DMA-complete timeouts.

### 6.2 Reliability mechanisms
| Mechanism | Detail |
|---|---|
| **IWDG** (LSI ~32 kHz, timeout ≈1–2 s) | Fed by the UI task (the one that must always make progress). A hung display write or deadlocked mutex → clean reset. Acquisition stall is caught earlier by DRDY timeout, so IWDG is the last line of defense, not the primary one. |
| **Sensor self-test** | At boot and on RECOVER: read WHO_AM_I (expect 0x33); verify DRDY toggles within N periods; optional static-range sanity check (output near zero at rest). |
| **DRDY watchdog counter** | If no DRDY observed for e.g. 16 expected periods → RECOVER state. This is the primary liveness check — it directly detects sensor clock loss, SPI corruption, or bus contention failure. |
| **Event log** | Fixed-size ring buffer (e.g., 64 entries: timestamp, code, counter) over UART/USB CDC; codes for SPI error, DRDY timeout, FFT init failure, display timeout. No dynamic allocation. |
| **CRC on telemetry** | If streaming is added later (BLE/USB), wrap each spectrum frame with CRC-16; the UI-only product needs no wire CRC but keeps it in the frame format for forward compatibility. |
| **Watched counters** | Expose: samples/sec measured vs 1600, ring overflow count, SPI error count — these are the production KPIs and belong in the event log / debug output. |

---

## 7. HAL Portability Structure (Project Layout)

No code here — only the structure that keeps the system portable across STM32 parts and boards:

```
vibro/
├─ Core/            CubeMX-generated: clock tree, HAL init, GPIO/SPI/DMA/TIM handles
├─ Drivers/         Vendor: STM32F4 HAL + CMSIS (untouched)
├─ Middlewares/     FreeRTOS port, CMSIS-DSP (arm_math) — framework-agnostic C
├─ App/
│  ├─ app_main.c        Task creation, FSM dispatch, IWDG feed policy
│  ├─ acq/lis3dsh.*     Register map, DRDY poll, DMA read sequence, self-test
│  ├─ dsp/frame_pipe.*  Ring buffer pop → window (const table) → rfft → peak
│  ├─ ui/st7735_drv.*   Thin HAL-SPI+DMA driver; chunked renderer API
│  ├─ bus/spi_arb.*     Mutex + chunk-size policy (§5), per-device SPI settings
│  └─ sys/health.*      Event log, DRDY watchdog, counters, FSM events
├─ Board/board_f407.h   THE ONLY board-specific file: pin map (PE3/PE4/PE5/PE6),
│                       SPI instance IDs, DMA stream numbers, memory section names
```

**Portability rules:**
1. All hardware access goes through `App/*` drivers; no direct register writes outside `Core/`.
2. Pin and peripheral identities live only in `Board/board_f407.h` — porting to another F4 board = one file + CubeMX regen.
3. Memory placement expressed as named sections (e.g., `.ccm_dsp`, `.dtcm_dma`) resolved by the scatter file, so CCM/D1 choices are build-time configuration, not code.
4. CMSIS-DSP and FreeRTOS stay in `Middlewares/` — zero Arduino dependencies anywhere; Adafruit GFX is deleted entirely.

---

## 8. Migration Plan (Arduino → HAL/FreeRTOS)

| Phase | Scope | Acceptance criteria | Est. effort |
|---|---|---|---|
| **0. Prototype fixes** (stay in Arduino, optional) | Allocate FFT scratch explicitly; precompute Hann table; stop pause/resume — run timer continuously with ping-pong buffers; add WHO_AM_I check | 1600 Hz continuous for ≥24 h without gaps; spectrum refresh >5 fps | 1–2 days |
| **1. Bare-metal HAL port** | CubeMX project: SPI1+DMA, TIM2 @6.4 kHz, DRDY-gated acquisition → ring buffer; super-loop runs DSP + chunked UI; CMSIS-DSP verified with sine-in/sine-out test | Measured sample rate 1600 ± <5 Hz over 1 h; jitter <10 µs (timestamp via GPIO toggle); FFT peak error ≤1 bin on known tone | 3–5 days |
| **2. FreeRTOS target** | Introduce tasks per §3.4, `spi_arb` mutex + chunking (§5), CCM/D1 placement per §4, IWDG + event log + FSM (§6) | All Phase-1 metrics hold under UI load; forced-fault tests (pull CS mid-frame, block DRDY) recover via RECOVER within 3 attempts; IWDG resets on induced hang | 5–8 days |
| **3. Hardening & validation** | Long-run soak (72 h), thermal check of SPI @10 MHz, 3-axis feasibility study if product requires it, telemetry/CRC format freeze | Zero unlogged faults in soak; KPI counters stable; documented memory map signed off | 3–5 days |

**Sequencing rationale:** Phase 1 de-risks the two hard problems (DRDY-gated DMA acquisition and CCM DSP placement) without RTOS variables; FreeRTOS is then a low-risk layering step because every task's contract was already proven in the super-loop.

---

## 9. Risk Register

| # | Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|---|
| R1 | LIS3DSH burst-read assumption wrong (DMA pattern yields junk bytes) | Medium | High — corrupt samples | Verify against datasheet in Phase 0/1; fallback = two sequential 2-byte DMA transfers per sample |
| R2 | CCM placement mistake → DMA faults on CCM address | Low | Fatal crash at boot | Scatter-file review + early DMA self-test writing to DTCM and reading back |
| R3 | Display chunking too coarse → sensor starvation reappears | Medium | Spectral gaps | Enforce ≤100 µs max transaction; monitor ring overflow counter in soak test |
| R4 | FreeRTOS stack under-sizing (UI task with float formatting) | Low | Hard fault | Size stacks from Phase-1 profiling + `xPortGetFreeHeap` margin check at boot |
| R5 | IWDG timeout too aggressive → spurious resets during UI refresh spikes | Medium | Unreliable field behavior | Tune to 2× worst measured frame time; feed only from UI task, never from ISR |
| R6 | Scope creep to 3-axis / streaming without memory re-plan | Low | DMA buffer overflow | D1 map in §4.1 reserves headroom (ring 8 KB + DMA ≤1 KB of 192 KB); revisit before Phase 3 freeze |

---

## 10. Bottom Line
Keep the DSP core (CMSIS-DSP rfft-1024) and the sensor configuration (ODR 1600 Hz, ±16 g, BDU=1) — they are sound. Replace everything around them: **DRDY-gated SPI-DMA acquisition into a DTCM ring buffer**, **FreeRTOS producer/consumer separation** so rendering can never touch the sample path, **CCM-resident FFT workspace with precomputed window tables**, **chunked SPI arbitration bounded to 100 µs transactions**, and an **IWDG + DRDY-watchdog + event-log fault architecture**. That converts a demo that pauses its own sensor into a device whose sampling integrity is provable under load.
