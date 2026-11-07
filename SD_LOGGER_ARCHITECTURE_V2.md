# SD Logger V2 — Architecture & Strategy (WeAct STM32F405RG)

**Target:** STM32F405RGT6, Cortex-M4F @ 168 MHz, **D1 SRAM = 192 KB** (@0x2000_0000), CCM RAM = 64 KB (@0x1000_0000, CPU-only — DMA cannot address it), Flash 1 MB.
**Sensor:** LIS3DSH on SPI1 (board not yet available → HIL phase: DAC→ADC loopback). **Storage:** MicroSD via hardware 4-bit SDIO + FatFs R0.12c. No display, no RTOS.

## 1. Executive Summary — V2 vs v1
- Storage path unchanged from `docs/SD_LOGGER_STRATEGY_v1_ORIGINAL.md`: native 4-bit SDIO (not SPI), FatFs over `HAL_SD`, triple buffering, chunked self-describing binary file, IWDG + FSM.
- **Corrected arithmetic** (v1 errors found in Step-2 audit): buffer fill time = **1.28 s** (2048/1600 Hz), not "128 ms"; worst-case stall headroom = **3 × 1.28 s ≈ 3.84 s**, not "384 ms" (factor-of-10 error, margin direction still correct); D1 SRAM = **192 KB**, not 128 KB; throughput = **9.6 KB/s** (1600 Hz × 3 axes × 2 B), not 9.4; TIM2 derivation = **84 MHz / (125 × 420) = 1600 Hz** (APB1 timer clock doubles to 84 MHz; PSC=124, ARR=419 — values were right in v1, the formula text was wrong).
- **Dropped from `docs/reference_vibro_architecture.md`:** ST7735 display + SPI arbitration (§5 there), FreeRTOS task table (§3.4), on-board CMSIS-DSP rfft-1024 in CCM — spectral analysis moves to PC (numpy/pandas). V2 is a bare-metal HAL super-loop: single producer (ADC/SPI DMA) + single consumer (main-loop SD writer); no locks needed.
- **Kept from reference doc:** D1-vs-CCM placement rule, IWDG as last line of defense + latched error codes with LED blink patterns, "sensor clock is master" principle (TIM2 @ 1600 Hz replaces DRDY polling).

## 2. Pin Map Reference
All pins per `docs/WEACT_F405_PINOUT.md` (single source of truth):

| Signal | Pin | Note |
|---|---|---|
| SDIO_D0–D3 | PC8, PC9, PC10, PC11 | 4-bit data bus, AF12 |
| SDIO_CMD / CK | PD2 / PC12 | CMD pull-up 100 kΩ to VDD33 |
| SD card detect (CD) | **PA8** | Active LOW: inserted → 0; EXTI falling/rising edge |
| User button (KEY) | PC13 | Active HIGH, EXTI rising edge |
| LED Blue | PB2 | HIGH = on (error blink patterns) |
| HSE / LSE | PH0–PH1 (8 MHz) / PC14–PC15 (32.768 kHz) | PLL → 168 MHz; RTC from LSE |
| SWD | PA13/PA14 | Debug only |

**⚠️ PA5 conflict — resolved:** HIL uses **PA5 = DAC_CH2**; production LIS3DSH needs **PA5 = SPI1_SCK** (pinout §6). The pinout doc's suggestion "move DAC to PA4" is rejected: **PA4 is reserved for LIS3DSH_CS**. Therefore in production the DAC peripheral must be **disabled entirely** (`MX_DAC_Init` removed, PA5 re-muxed to SPI1_SCK AF5). HIL-only pins (DAC PA5, ADC PA0–PA2) are freed after migration; nothing else collides with SPI1 (PA4–PA7), SDIO (PC8–PC12/PD2), or CD/button/LED.

## 3. Critique of Prior Audits
- `docs/reference_vibro_architecture.md`: survives = SPSC/ring-buffer producer-consumer, D1-vs-CCM rule, IWDG + FSM + latched errors; obsolete = display arbitration, FreeRTOS, CCM FFT workspace (see §1).
- `docs/SD_LOGGER_STRATEGY_v1_ORIGINAL.md` arithmetic errors: fill time 128 ms → **1.28 s**; headroom 384 ms → **3.84 s** (factor-of-10 error, margin direction still correct); "D1 = 128 KB" → **192 KB**; TIM2 formula text wrong (values PSC=124/ARR=419 are right).
- `docs/project_history.md` lessons: UART@921600 + sprintf streaming lost data above ~200 Hz → standalone SD logging justified; **ADC DMA "Continuous Requests" must be ENABLE or DMA stops after the first scan** (silent data loss); burst-read abandoned in Arduino due to SPI wrapper bug — on F405 verify LIS3DSH auto-increment DMA read against datasheet, fallback = per-register reads (`readReg(0x28)`, `readReg(0x29)`).

## 4. HIL Test Plan (DAC → ADC loopback)
**Signal chain:** DAC_CH2 (PA5), circular DMA LUT of 10 samples (12-bit sine, `dac_lut[]` in `src/main.c`) → jumper wires PA5→PA0/PA1/PA2 → ADC1 scan mode.

- **TIM2 @ exactly 1600 Hz:** APB1 timer clock = 84 MHz; PSC=124 (÷125), ARR=419 → 84e6/(125×420) = 1600 Hz. Update event triggers ADC scan and DAC DMA step.
- **ADC1:** PA0/PA1/PA2, 12-bit, scan mode, external trigger TIM2 TRGO rising edge; clock PCLK2 (84 MHz)/4 = 21 MHz; sample time 144 cycles ≈ 6.9 µs/channel → full 3-channel scan ≈ **20.7 µs ≪ 625 µs period** (no overrun). DMA2_Stream0, circular, into `adc_buffer[3]`.
- **⚠️ Pitfall (`docs/project_history.md`):** CubeMX "DMA Continuous Requests" must be **ENABLE**. Current `src/adc.c` has `hadc1.Init.DMAContinuousRequests = DISABLE` → DMA stops after the first scan cycle; fix before the first HIL run.
- **Current gap:** `main.c` writes raw 6 B records with no header/chunks — the V2 format below applies already in HIL so the PC parser is identical for both phases.

**Binary file format (little-endian, same in HIL and production):**
```
HEADER (fixed 256 B): magic "VIBL" | version u16 = 1 | sample_rate u32 = 1600
  | axes u8 = 3 | bytes_per_sample u8 = 6 | sensor[16] ("HIL-DAC-ADC"/"LIS3DSH")
  | range_g i8 | gain f32 | offset_x/y/z int16×3 | start_ts u64 (RTC ms)
  | record_count u32 | header_crc u32
CHUNKS: [chunk_hdr][n × record] ...   chunk_hdr = seq_number u32 | sample_count u16
RECORD: int16 x, y, z                  (HIL: 3 ADC channels; production: X/Y/Z)
TRAILER (clean stop only): end_ts u64 | total_records u32 | file_crc32 u32
```
Note: `_FS_NORTC=1` in `include/ffconf.h` only fixes FAT directory timestamps; the VIBL header timestamp comes from RTC (LSE) via `HAL_RTC_GetTime`.

**PC parser spec:** verify magic/version/header_crc → iterate chunks in order → **gap = seq_number jump > 1** (report dropped samples, never assume contiguity) → on EOF: if trailer present and `file_crc32` matches payload → "clean"; else "truncated at chunk k" (power-loss case). Output: per-chunk counts, gap list, total records.

**1-hour zero-loss validation:** expected = 1600 × 3600 = **5,760,000 records ≈ 34.56 MB**; pass criteria: trailer CRC OK, zero seq gaps, record_count == 5,760,000 ± <0.1 % (RTC drift), file size matches; repeat 3× with SD card removed mid-run to verify gap/truncation detection.

## 5. Concurrency Model — bare-metal + DMA (no RTOS)
- **Producer:** ADC1 (HIL) / SPI1 RX DMA (production) runs continuously, zero CPU per sample. TIM2 ISR is empty in HIL; in production it only arms/verifies the 6-byte burst read (<0.5 % CPU).
- **Consumer:** main-loop writer drains READY buffers via `HAL_SD` + FatFs. Single-threaded → no locks, no mutex (`_USE_MUTEX=0`, `_FS_REENTRANT=0` in `include/ffconf.h`).
- **Latency budget (worst case):** SDIO write of 12 KB ≈ a few ms card time at ~21 MHz CK (`src/sdio.c`: ClockDiv=0 → APB1 42 MHz ÷ 2); worst stall ≤ 200 ms; IWDG timeout ≈ 1.0–1.5 s (LSI ~32 kHz, prescaler /8, reload 4095) ≫ 200 ms → no spurious reset; a real hang trips it.
- **Current vs target:** `src/diskio.c` uses blocking `HAL_SD_ReadBlocks/WriteBlocks` (1000 ms timeout); V2 migrates disk_write to non-blocking + SDIO-DMA completion callback — the dedicated SDIO DMA engine consumes no generic DMA stream, freeing all streams.
- **Why not FreeRTOS:** only two actors (DMA producer, loop consumer); worst-case blocking op is bounded and IWDG-covered; RTOS would add ~10–20 KB RAM + tick jitter for zero benefit at 9.6 KB/s sustained load (CPU <15 %).

## 6. Buffer Management — triple buffering
- **N = 2048 samples × 6 B = 12,288 B = exactly 24 sectors** (512 B) per buffer → FatFs writes are sector-aligned, no partial-sector waste.
- **Fill time: 2048 / 1600 Hz = 1.28 s** (v1's "128 ms" was a factor-of-10 error).
- Three buffers rotate FILLING → READY → WRITING; roles never collide, so an in-flight SD write can't tear the buffer being filled.
- **Headroom:** during one worst-case 200 ms stall, up to two more full buffers (≈ 2 × 1.28 s) accumulate → total absorption ≈ **3.84 s** ≫ 200 ms → zero loss under normal stalls.
- **Overflow policy (all 3 full):** drop the oldest READY buffer but emit its `seq_number` as a gap marker in the next chunk header; count dropped samples; keep logging — never reset.

## 7. Memory Map (F405RG: D1 = 192 KB, CCM = 64 KB)
| Item | Size | Location | Rationale |
|---|---|---|---|
| Triple sensor buffers (3 × 12,288 B) | **36,864 B** | D1 | DMA master cannot address CCM |
| FatFs: FS struct + FIL window buffer + FAT cache | ~4 KB | D1 | `_FS_TINY=0` → per-FIL 512 B sector buffer |
| `HAL_SD` card-info/IO buffers | ~1–2 KB | D1 | SDIO has dedicated DMA engine (no generic stream consumed) |
| Stack + globals (`adc_buffer`, LUT, FSM state) | ~8–16 KB | D1 | — |
| **D1 total** | **~50–60 KB of 192 KB** | | headroom for N=4096 (3 × 24 KB = 72 KB) if more margin wanted |
| CCM 64 KB | reserved | CCM | CPU-only; future on-board DSP (CMSIS twiddles/FFT scratch) — never DMA targets |

## 8. State Machine & Error Handling
```
BOOT ──ok──▶ IDLE ──button(PC13, debounced ~30 ms)──▶ RECORDING ──button──▶ FLUSHING ──clean stop──▶ IDLE
  │ boot fail / SD missing / sensor missing                    │ f_sync + trailer OK
  ▼                                                            ▼
ERROR (latched code → PB2 blink pattern) ◀── SD_LOST (PA8 EXTI rising edge mid-write) / write retry ×N failed
```
- **BOOT:** clocks, GPIO, SDIO+FatFs mount (`f_mount`), IWDG start; HIL: DAC/ADC/TIM2; production: SPI1 + LIS3DSH self-test (WHO_AM_I = 0x33). Fail → ERROR.
- **IDLE:** Stop mode (RAM retained); wake on PC13 EXTI or RTC tick; SDIO/SPI clocks gated for battery.
- **RECORDING:** producer/consumer running; IWDG fed every loop iteration **and** inside `disk_read`/`disk_write` (`src/diskio.c` already does this — keep it, since blocking HAL_SD can stall up to the 1000 ms timeout); **`f_sync` after each chunk write** (per-buffer commit → power loss corrupts at most the in-flight buffer).
- **PA8 EXTI:** falling edge = card inserted (re-init), rising edge during RECORDING = SD_LOST → stop sampling, ERROR(SD_LOST); already-flushed chunks stay committed.
- **FLUSHING:** freeze TIM2 (stop producer) → drain READY/WRITING buffers → write trailer + `file_crc32` → `f_sync` → IDLE. Guarantees a clean parseable file on user stop.
- **ERROR codes** (PB2 blink): 1 = SD missing, 2 = FatFs mount fail, 3 = sensor fault, 4 = buffer overflow latched, 5 = write retry exhausted.

## 9. Migration Plan — HIL → LIS3DSH
1. **Disable ADC/DAC:** remove `MX_ADC1_Init`/`MX_DAC_Init`, free DMA2_Stream0; PA5 re-muxed to SPI1_SCK (AF5). DAC is not moved to PA4 — **PA4 stays reserved for LIS3DSH_CS** (software-controlled GPIO, initial HIGH = deselected).
2. **Enable SPI1:** full-duplex master, NSS software, 8-bit, clock ≤ 10 MHz (LIS3DSH max); init sequence per `docs/project_history.md`: soft reset (CTRL_REG3 → 0x80), ODR=1600 Hz + BDU=1 + XYZ=1 (CTRL_REG4 0x20 → 0xBF), ±16 g (CTRL_REG5 0x24 → 0x20).
3. **Capture path:** TIM2 update ISR arms SPI RX DMA of one burst read (auto-increment ODR registers, 6 B = int16 X/Y/Z) into the FILLING buffer; verify burst-read behavior against datasheet first — fallback is two sequential 2-byte register reads (`0x28`, `0x29`), which doubles SPI time to ~10 µs/sample (still <1 % CPU).
4. **Unchanged:** FatFs + `src/diskio.c` + triple-buffer manager + file format + FSM — only the producer front-end swaps, exactly as planned in `docs/project_history.md`.
5. **Acceptance after migration:** measured rate 1600 ± <5 Hz over 1 h (via record count vs RTC), zero seq gaps, jitter check by toggling a spare GPIO per sample if needed; IWDG never trips during normal SD stalls.
