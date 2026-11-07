# High-Speed Vibration Data Logger (V2) — Architecture & Strategy

**Target:** Battery-powered, standalone 3-axis vibration logger @ **1600 Hz**, zero sample loss over hours.
**MCU:** STM32F405RGT6 (WeAct Studio 64-Pin Mini Core Board) — Cortex-M4F @ 168 MHz, 128 KB D1 SRAM + 64 KB CCM RAM, hardware FPU.
**Sensor:** LIS3DSH (external, SPI1). **Storage:** onboard MicroSD slot. **Control:** BOOT/User button. **Display:** none.

---

## 1. Project Overview (V2)

| Aspect | V1 (retired) | V2 (this design) |
|---|---|---|
| Board | STM32F407 Discovery + display | WeAct F405RGT6 mini core, **no display** |
| Power | Bench / USB | **Battery**, portable, efficiency-first |
| Sampling | ad-hoc | Deterministic 1600 Hz via Timer+DMA |
| Storage | — | Onboard SD slot (native **SDIO**) |
| Reliability | basic | IWDG + crash-safe chunked file format |

**Non-negotiable goals:** (a) never drop a sample under normal SD stalls; (b) survive power loss without corrupting prior data; (c) minimize battery draw between recordings; (d) produce a PC-parseable, self-describing binary file.

---

## 2. Hardware Topology & Framework Choice

### 2.1 Confirmed board wiring (from `WEACT_F405_PINOUT.md` — the project's single source of truth)
The onboard SD slot is wired to the **native STM32 SDIO peripheral**, not SPI:

| Signal | Pin | Role |
|---|---|---|
| SD_D0 / D1 / D2 / D3 | PC8 / PC9 / PC10 / PC11 | 4-bit data bus |
| SD_CMD | PD2 | Command line (pull-up to VDD33 via 100 kΩ) |
| SD_CK | PC12 | Clock |
| Card detect (CD) | PA8 | Presence switch — **Active LOW** (card inserted → reads 0) |

Sensor **LIS3DSH** on **SPI1**: SCK = PA5, MISO = PA6, MOSI = PA7, chip-select = **PA4** (software-controlled). User button = **PC13** via EXTI (**Active HIGH**, pressed → 3.3 V). Status LED Blue = **PB2** (lights on HIGH; used for the §7 error blink patterns). No display peripherals are used, freeing their clocks for power savings.

### 2.2 Framework decision: **STM32 HAL (CubeMX-generated) vs Arduino** — we choose **STM32 HAL**

Evaluated against the real constraints (deterministic 1600 Hz sampling, DMA buffer placement in D1 SRAM, SDIO-DMA offload, IWDG tuning):

| Criterion | Arduino (STM32duino) | **STM32 HAL + CubeMX** ✅ |
|---|---|---|
| Deterministic Timer/DMA/NVIC setup | Abstracted; less control over priorities & DMA stream/channel assignment | Generated correctly by CubeMX — exact NVIC priority, DMA stream/channel, SDIO config |
| SDIO driver (card init + DMA) | SPI-biased libs; weak/absent native SDIO path | `HAL_SD` supports blocking **and** non-blocking+DMA — exactly what producer-consumer needs |
| CCM-vs-D1 buffer placement | Opaque; hard to force buffers into D1 SRAM | Explicit section attributes + clean linker control |
| ISR / callback model | `loop()`-centric, harder to reason about timing | Clean HAL callbacks (`HAL_SPI_RxCpltCallback`, `HAL_SD_TxCpltCallback`) map 1:1 onto our state machine |
| Footprint / overhead | Moderate (unnecessary here) | Negligible on a 1 MB/128 KB part |
| Maintainability & reliability | Good | **Best balance** — generated, well-tested driver layer |

**Rationale:** The F405 has ample RAM/flash, so minimal footprint is *not* the goal — **reliability and throughput are**. CubeMX eliminates the most error-prone setup (clock tree, GPIO AF muxing, NVIC priorities, DMA stream/channel assignment, SDIO peripheral config). `HAL_SD` gives both blocking and non-blocking+DMA modes, which is exactly what the producer-consumer design needs. The FPU lets us do float calibration in ISR context cheaply. WeAct ships HAL examples for this exact board (RTC, MSC), confirming HAL is the native path. **Decision: STM32 HAL + CubeMX, C.**

**Build-flag rationale (see `platformio.ini`):**
- `-DUSE_HAL_DRIVER -DSTM32F405xx` — required by the CubeMX-generated code to select the correct HAL driver set and device header.
- `-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard` — enables the Cortex-M4 **hardware FPU** (single-precision VFPv4) so float calibration in ISR context is cheap; without these, float ops fall back to slow software emulation.
- `-Ofast -ffast-math -funroll-loops` — speed-first optimization for the deterministic 1600 Hz sampling path. `-ffast-math` relaxes IEEE strictness (no NaN/Inf-dependent logic in this design), which is acceptable here; drop it if you ever need bit-exact float behavior.
- `-DCMSIS_DSP` — exposes CMSIS-DSP intrinsics for any future signal-processing work on the captured samples.

### 2.3 STM32CubeMX configuration (GUI steps)

Generate the project in **STM32CubeMX** targeting **STM32F405RGT6**, then import the generated code into this PlatformIO workspace (`framework = stm32cube`). Follow these steps exactly — they encode every peripheral decision from §2–§8.

> **Pin source of truth:** all pin assignments below come from `WEACT_F405_PINOUT.md` (the project's single source of truth, verified against the board schematics). Do not deviate without updating that file first. Note the polarity conventions: **User button PC13 is Active HIGH** (pressed → 3.3 V), **SD card-detect PA8 is Active LOW** (card inserted → reads 0), and **LED Blue PB2 lights on HIGH**.

1. **New Project → MCU Selector:** search `STM32F405RG`, select it, click **Start Project**.
   - *Project Manager tab:* Project Name = `SDlogger405RG`; Toolchain/IDE = **Makefile** (or any — we build via PlatformIO); check **"Generate peripheral initialization as a pair of 'c'/'h' files"**; uncheck "Use CMSIS Compiler specific headers" if you want portable code.
2. **RCC → High Speed Clock Source:** `HSE - Crystal/Ceramic Resonator` (8 MHz on PH0/PH1). **System Clock Source:** `PLLCLK`. **HCLK** = 168 MHz, **APB1** = 42 MHz, **APB2** = 84 MHz. *(Recording may later scale HCLK down to ~96–120 MHz for power — see §6.1; the base tree stays at 168 MHz.)*
3. **GPIO — SD card (onboard slot):** set `PC8, PC9, PC10, PC11` → **GPIO_Output** (SDIO data D0–D3), `PD2` → **GPIO_Output** (CMD), `PC12` → **GPIO_Output** (CK). *(CubeMX auto-assigns the SDIO alternate function when you enable the peripheral below — these pins are listed for reference.)* Set `PA8` → **GPIO_Input**, pull-up, no external interrupt yet (card-detect is optional; we probe via init instead).
4. **SDIO1:** enable it. Mode = **SD mode 4-bit**; Data Bus Width = **4 bits**; Pull-up on CMD/DATA enabled. This creates the `HAL_SD` driver and wires the dedicated SDIO-DMA engine (no generic DMA stream consumed — see §8.2).
5. **SPI1:** enable it, Mode = **Full-Duplex Master**, NSS = **Software** (chip-select is a GPIO we toggle), Data Size = **8 Bits**, prescaler set for ≤ the LIS3DSH max SPI clock (~10–16 MHz → choose a safe divider). This drives the sensor capture DMA.
6. **GPIO — Sensor chip-select:** `PA4` → **GPIO_Output**, initial level **HIGH** (deselects the LIS3DSH; we drive it LOW to select in software). SPI1 pins PA5/PA6/PA7 are auto-assigned when SPI1 is enabled above.
7. **GPIO — User button & status LED:** `PC13` → **GPIO_Input**, pull-down, with an **External Interrupt on Rising Edge** (button is Active HIGH: pressed → 3.3 V). `PB2` → **GPIO_Output**, initial level **LOW** (LED Blue off; we set it HIGH to light — used for the error blink patterns in §7).
8. **Timer2:** enable as **General Purpose Timer**. Prescaler + auto-reload chosen so the update event fires at exactly **1600 Hz** (e.g., with APB1 = 42 MHz: PSC = 124, ARR = 419 → 42 MHz / 40 / 32 = 1600 Hz). Enable the **Update interrupt**. This is the master sampling clock (§4.1).
9. **RTC:** enable it and set its **clock source to LSE** (the external 32.768 kHz crystal on PC14/PC15) — *not* LSI, per the pinout note; this gives an accurate `start_timestamp` for the file header (§5.2).
10. **NVIC Settings:** enable and set priorities (lower number = higher priority):
    - `TIM2`  — Priority **0** (highest; sampling tick — must never be delayed by SD writes)
    - `SPI1`  — Priority **1** (sensor DMA capture)
    - `SDIO1` — Priority **2** (lowest of the three; SD card writes can wait a few microseconds for sampling)
    - `EXTI15_10` (PC13 button) — Priority **3**
    - `SysTick`/PendSV left at defaults. *(IWDG needs no NVIC entry — it's a reset source, not an interrupt.)*
11. **System Core → Cortex-M4:** enable the **FPU** (single-precision). This is what makes `-mfpu=fpv4-sp-d16 -mfloat-abi=hard` in `platformio.ini` valid and fast.
12. **IWDG (Independent Watchdog):** enable it under **System Core → IWDG**. Prescaler + reload set for a **~1–2 s** timeout (§6.2) — comfortably above the worst ~200 ms SD stall, short enough to catch a real hang. It runs on LSI and stays alive in Stop mode.
13. **Clock Configuration tab:** verify HCLK = 168 MHz, APB1 = 42 MHz, APB2 = 84 MHz; confirm no clock-domain warnings (SDIO/SPI/TIM all on valid buses).
14. **Project Manager → Code Generator:** set Targeted IDE to your preference, then click **GENERATE CODE**.
15. **Import into PlatformIO:** copy the generated `src/` + `include/` (CubeMX output) into this workspace; keep our application logic in separate files so CubeMX regeneration never clobbers it. Build with `pio run`. FatFs is bundled with the HAL distribution used by `framework = stm32cube`, so no extra dependency is needed.

> **Regeneration safety:** always place custom code (state machine, buffer manager, file-format writer) in files *outside* CubeMX's generated set, and keep a copy of your `.ioc` — regenerating from the `.ioc` must never overwrite application logic.

---

## 3. Storage Interface Analysis — SDIO vs SPI

**On this specific board the slot is hardwired to SDIO**, so SPI is not even an option for the onboard connector. The analysis below justifies why that is the correct outcome and what we would lose on a hypothetical SPI-wired variant.

| Factor | **SDIO (this board)** ✅ | SPI (hypothetical) |
|---|---|---|
| Data bus width | 4-bit (D0–D3) | 1-bit full-duplex |
| Peak clock / throughput | ~25 MHz HS mode; realistic sustained **10–30 MB/s** | ≤42 MHz but lower effective write rate |
| CPU overhead during transfer | **~0%** — SDIO has its own dedicated DMA engine | Higher unless generic-DMA driven, still per-byte protocol handling |
| Card init / CMD-ACMD sequences | Handled in hardware peripheral | Software driver stack (more code, more IRQ load) |
| Generic DMA streams consumed | **None** (dedicated SDIO DMA) | Consumes a generic DMA stream |

**Throughput requirement check:** 1600 Hz × 3 axes × 2 B = **9.4 KB/s**. Even the slowest class-4 card sustains >1 MB/s, so raw bandwidth is *not* the constraint — **latency spikes (50–200 ms blocking writes) and CPU overhead are.** SDIO + its dedicated DMA solves both: a 16 KB write takes ~a few ms of card time with essentially zero CPU involvement. **Verdict: SDIO wins decisively** on this board, and it is the only viable path for the onboard slot anyway.

---

## 4. Concurrency & Buffer Management (Producer–Consumer, Triple Buffering)

### 4.1 Data flow
```
LIS3DSH ──(SPI1 RX DMA)──► [Buffer A] ──► READY ──► SDIO-DMA write ──► FATFS file
   ▲ timer tick @1600 Hz        (FILLING)      (WRITING)
   │
Timer2 update event = master sampling clock (deterministic, IRQ-driven)
```

- **Producer:** Timer2 generates an exact 1600 Hz update interrupt. Each tick arms a **SPI1 RX DMA** transfer that captures one LIS3DSH burst read (auto-incrementing ODR registers → 6 bytes = int16 X/Y/Z). DMA moves the raw bytes into the current buffer with **zero CPU involvement**.
- **Consumer:** main-loop SD writer drains completed buffers to the card via `HAL_SD` non-blocking + SDIO-DMA, then syncs.

### 4.2 Triple-buffering scheme (why not ping-pong)
Three fixed-size buffers rotate through three roles that must never collide:

| Role | Meaning |
|---|---|
| **FILLING** | Sensor DMA currently writing this buffer |
| **READY** | Full, awaiting SD write |
| **WRITING** | SDIO-DMA transferring to card |

With triple buffering these three roles are always distinct. When the sensor DMA completes a buffer → mark it READY and retarget DMA to the next FILLING buffer; the writer picks up any READY buffer independently. Ping-pong (2 buffers) would let an in-flight write collide with the fill during a stall — unacceptable.

### 4.3 Buffer sizing & stall headroom
- **N = 2048 samples/buffer** → 2048 × 6 B = **12 KB per buffer**.
- Time to fill one buffer: 2048 / 1600 Hz = **~128 ms**.
- Triple-buffer total in D1 SRAM: **3 × 12 KB = 36 KB**.

**Worst-case absorption:** while one buffer is being written (worst SD stall ≈ 200 ms), the sensor can fill up to **two more buffers = ~256 ms** of additional data before overflow. Combined with the in-flight buffer, total headroom ≈ **~384 ms**, comfortably exceeding a single 200 ms stall → **no loss under normal stalls.**

> Scaling option: if more margin is wanted, raise N to 4096 (24 KB × 3 = 72 KB) — still within the 128 KB D1 budget. Base recommendation stays at N=2048 for lower per-write latency.

### 4.4 DMA buffer placement constraint
All three sensor buffers **and** the SDIO/FATFS write staging buffer **must reside in D1 SRAM (0x2000_0000–0x2003_FFFF)**. The generic DMA controller and SDIO-DMA **cannot access CCM RAM (0x1000_0000)** — placing them there would silently corrupt transfers. Enforce via explicit linker section placement; reserve CCM for non-DMA data only.

---

## 5. File Format & Data Structure

### 5.1 Storage layer: **FATFS (FatFs) over `HAL_SD`**
Chosen over raw sector writes because it yields a standard, PC-readable filesystem with no bespoke recovery tooling, delegates wear-leveling to the card's FTL, and lets us commit/flush cleanly per chunk. The *payload* is our own compact binary container — **not CSV** (CSV at 1600 Hz × 3 axes would be ~5–8× larger and slow to parse). Binary int16 is small, fast to write, and trivially loaded by numpy/pandas on the PC.

### 5.2 Binary container layout (little-endian)
```
FILE HEADER (fixed, e.g., 256 B):
  magic[4]          = "VIBL"            // identifies type + version
  version           u16   = 1
  sample_rate_hz    u32   = 1600
  num_axes          u8    = 3
  bytes_per_sample  u8    = 6           // int16 x,y,z
  sensor_model[16]         = "LIS3DSH"
  range_g           i8                // ±2/±4/±8/±16 g
  gain              f32               // LSB → g calibration factor
  offset_x/y/z      int16 ×3          // per-axis zero bias (optional)
  start_timestamp   u64               // RTC-based epoch (ms)
  record_count      u32               // total samples in file
  header_crc        u32

PAYLOAD — sequence of self-describing CHUNKS:
  [ chunk_hdr ][ n × record ] ...
    chunk_hdr : seq_number u32 | sample_count u16   // gap detection
    record    : int16 x, int16 y, int16 z           // one sample

FILE TRAILER (written on clean stop):
  end_timestamp   u64
  total_records   u32
  file_crc32      u32               // integrity over payload
```

**Why chunked:** each buffer is written as a chunk carrying its **sequence number**. The PC parser detects dropped data by gaps in `seq_number` — drops are explicit and self-describing, never silently assumed contiguous. This makes the format crash-safe (see §6).

---

## 6. Power Management & Reliability

### 6.1 Battery strategy
- **During RECORDING:** Sleep/Stop modes are **not viable** — we sample every 625 µs and Stop-mode wake latency is too long/unpredictable, which would drop samples. Keep the MCU active; the dominant draw is SD write bursts + SPI reads, not CPU idle. Because DMA does the heavy lifting (CPU < ~15% sustained), we can **scale HCLK down** from 168 MHz to ~96–120 MHz during recording to cut dynamic power while still meeting the sampling+write budget.
- **During IDLE:** this is where battery life is won. Gate SPI1/SDIO clocks, then enter **Stop mode** (RAM retained → calibration/state preserved; wake in µs). Use **Standby** only if near-zero draw is required and full re-init on button press is acceptable. A low-frequency RTC tick can optionally drive a very-low-rate heartbeat log while lightly sleeping.
- **Policy:** IDLE = Stop mode (clocks gated); RECORDING = active, possibly clock-scaled. This preserves sample integrity far better than per-sample sleep.

### 6.2 Watchdog strategy (IWDG)
Use the **Independent WDT** — it runs on LSI (~32 kHz), independent of the main clock and still running in Stop mode, so it self-recovers even if the main clock fails (ideal for a battery device).

- **Timeout:** set to comfortably exceed the worst legitimate blocking op. Worst case = one SD write (~200 ms) + margin → **~1–2 s**. A normal 200 ms stall never trips it; a genuine hang (seconds of no progress) does.
- **Feeding:** because we use **non-blocking `HAL_SD` + DMA with completion callbacks**, the main loop stays responsive and feeds IWDG every iteration naturally — there is no long unattended block to cause spurious resets. Any bounded wait loop also feeds the WDT inside it. This eliminates the classic "WDT reset during SD write" failure mode by design.

### 6.3 Graceful recovery (power loss / reset)
On any reset, boot re-initializes and inspects the last file:
- A **clean** file has a valid trailer + `file_crc32`. An **incomplete** file lacks it.
- Recovery rule: keep all chunks whose `seq_number` is contiguous from 0; discard the tail after the first gap/missing chunk. Because data is committed per-chunk with seq numbers, prior data is never corrupted and dropped samples are bounded to at most one buffer (~128 ms) at the instant of power loss.

---

## 7. State Machine & Error Handling

```
        ┌────────┐   boot ok    ┌───────┐  button  ┌────────────┐  stop  ┌───────────┐
        │ BOOT   │────────────►│ IDLE  │─────────►│ RECORDING  │───────►│ FLUSHING  │
        └────────┘             └───────┘◄─────────┴────────────┘       └───────────┘
              │ boot fail            ▲   wake (button/RTC)                 │ clean stop
              ▼                      └─────────────────────────────────────┘
           ┌────────┐  recoverable: re-init on next button press
           │ ERROR  │  unrecoverable: hold until power cycle
           └────────┘
```

| State | Behavior |
|---|---|
| **BOOT** | Init clocks, GPIO, SPI1 + LIS3DSH (read ID reg to confirm presence), SDIO + FATFS mount, start IWDG. Sensor or SD missing → ERROR with a code (LED blink pattern; no display). |
| **IDLE** | MCU in Stop mode; wake on button EXTI or RTC tick. Button debounced here. Lowest power state. |
| **RECORDING** | Timer+DMA sampling active, triple-buffer producer-consumer running, SD writer draining buffers, IWDG fed each loop iteration. Button → FLUSHING. |
| **FLUSHING** | Freeze the sampling timer (stop producer), drain all READY/WRITING buffers to SD, write trailer + CRC, `f_sync`, return to IDLE. Guarantees a clean parseable file on user Stop. |
| **ERROR** | Latched fault with code; LED blink pattern encodes it. Recoverable (SD removed) → re-init on next button press; unrecoverable (sensor gone) → hold until power cycle. |

### Error handling specifics
- **Buffer overflow** (all 3 buffers full): set a `gap` flag, drop the oldest READY buffer but record its seq so the PC parser sees the gap; **do not reset** — keep logging. Count total dropped samples for the report.
- **SD card removed mid-write:** SDIO DMA error / CMD timeout caught in callback → ERROR(SD_LOST), stop sampling to avoid unbounded overflow, re-init on next button press (or auto-retry with backoff). Already-flushed chunks remain committed via per-chunk FATFS sync.
- **SD write failure (CRC/timeout):** retry the same buffer up to N times; if still failing → ERROR. Never silently drop — data is either written or explicitly marked as a gap.

### Button control (non-blocking)
EXTI on the BOOT/User pin with hardware RC debounce + software timestamp debounce in the main loop (ignore re-triggers within ~30 ms; require stable level for ~20–50 ms before accepting). The EXTI only sets a flag; the state machine polls it and applies the transition — **no `delay()` in the handler**. Optional: short-press = start/stop toggle, long-press = force flush + safe stop.

---

## 8. Resource Analysis

### 8.1 RAM budget (D1 SRAM = 128 KB)
| Item | Size | Location |
|---|---|---|
| Triple sensor buffer (3 × 12 KB) | **36 KB** | D1 (DMA-accessible) |
| FATFS working/sector buffer + FS structs | ~12 KB | D1 (SDIO-DMA-backed writes) |
| `HAL_SD` internal buffers | ~4 KB | D1 |
| Stack + heap + globals | ~8–16 KB | D1 |
| **Total** | **~60–70 KB** | fits 128 KB with headroom to scale N up |

CCM RAM (64 KB) reserved for non-DMA data / large lookup tables only.

### 8.2 DMA channels
- **SPI1_RX → DMA2_Stream0_Channel3** — sensor capture (one generic stream).
- **SDIO TX/RX → dedicated SDIO DMA engine** — consumes **no** generic DMA stream and offloads the CPU entirely during card writes. This frees all other generic streams for future use.

### 8.3 CPU load estimate @ 1600 Hz
| Activity | Cost |
|---|---|
| Timer ISR (arm/verify DMA, bump index) | few hundred cycles/tick → **<0.5%** |
| SPI RX DMA-complete handler (int16 convert + append; FPU for calibration) | handful of instructions/sample → **<2%** |
| SD writer (main loop) | mostly waiting on completion flags; spikes to ~5–10% during a burst, near-zero sustained |
| **Total sustained** | **well under ~15%** → large margin; can run at reduced clock for power savings |

### 8.4 Key design invariants (checklist)
- [ ] All DMA buffers placed in **D1 SRAM**, never CCM.
- [ ] Sampling driven by **Timer2 @ exact 1600 Hz** + SPI1 RX DMA — not main-loop timing.
- [ ] SD writes are **non-blocking + SDIO-DMA**; IWDG fed every loop iteration (no long unattended blocks).
- [ ] Triple buffering gives **~384 ms** worst-case stall absorption (> 200 ms SD stall) → no loss under normal stalls.
- [ ] Chunked file format with seq numbers + trailer CRC → crash-safe, gap-detectable, PC-parseable.
- [ ] IDLE = Stop mode (battery); RECORDING = active/clock-scaled (sample integrity).
