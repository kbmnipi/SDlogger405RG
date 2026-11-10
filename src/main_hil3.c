/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_hil3.c
  * @brief          : HIL-3 test for SD Logger V2 (WeAct STM32F405RG, FatFs R0.12c).
  *
  * Purpose: end-to-end signal chain validation WITH the real LIS3DSH sensor
  * (on the Discovery board; F407 held in reset, NRST->GND):
  *   TIM2 @ exactly 1600 Hz fires an update interrupt; each tick reads one
  *   X/Y/Z sample from the LIS3DSH over SPI1 (blocking burst read with
  *   auto-increment) and appends it to a ping-pong buffer. When half of the
  *   buffer is full, buf_ready is set (same semantics as HIL-2); the main loop
  *   commits READY halves to SD as VIBL chunks through vibl_write_chunk().
  *
  * Pin map per docs/WEACT_F405_PINOUT.md:
  *   PA4 = LIS3DSH_CS (software driven, active LOW) -> Discovery PE3
  *   PA5 = SPI1_SCK (AF5), PA6 = SPI1_MISO (AF5), PA7 = SPI1_MOSI (AF5)
  *   PB2 = LED Blue, Active HIGH
  *
  * Timing budget:
  *   TIM2: 84 MHz / ((124+1)*(419+1)) = exactly 1600 Hz -> period 625 us.
  *   One SPI burst read (7 bytes @ 5.25 MHz) ~ 13 us << 625 us: no sample loss.
  *   One ping-pong half = 2048 samples = 640 ms of signal.
  *   Chunk commit: f_write(12 KiB) + f_sync <= ~250 ms worst case per 1.28 s (~20%).
  *   If a commit ever exceeds the half period, new samples overwrite the uncommitted
  *   half (same circular semantics as HIL-2 DMA).
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "ff.h"
#include "sd_writer.h"
#include "iwdg.h"
#include "rtc.h"
#include "sdio.h"
#include "gpio.h"
#include "tim.h"
#include "lis3dsh.h"

/* Private variables ---------------------------------------------------------*/
FATFS fs;

/* Ping-pong sensor buffer: 2 x 2048 samples x 3 axes (int16) = 12288 halfwords
 * = 24 KiB, static. Layout per scan: [x][y][z] interleaved — exactly the VIBL
 * on-disk record layout, so chunks are written zero-copy. */
static int16_t pingpong_buf[VIBL_SAMPLES_PER_CHUNK * 2u * 3u];

/* Write index inside pingpong_buf (ISR context). Resets to 0 after buffer B. */
static volatile uint32_t sample_idx = 0u;

/* Half/complete flags: set in the TIM2 ISR, consumed by the main loop ONLY.
 * Values: 0 = none, 1 = buffer A READY, 2 = buffer B READY (same as HIL-2). */
static volatile uint8_t buf_ready = 0u;

/* Latched sensor read failure (ISR context) -> error loop in main. */
static volatile uint8_t lis3dsh_err = 0u;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void hil3_error_loop(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration --------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals (HIL-3: no ADC/DAC/DMA). */
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_TIM2_Init();

  /* 1. Bring up the LIS3DSH on SPI1 (WHO_AM_I must be 0x3F). */
  if (lis3dsh_init() != 0) {
    hil3_error_loop();
  }

  /* 2. Mount the SD card (format=1: create FAT if the card is unformatted). */
  if (f_mount(&fs, "0:", 1) != FR_OK) {
    hil3_error_loop();
  }

  /* 3. Open the VIBL file: LIS3DSH sensor id, 16 g range, gain = 100 LSB/g. */
  if (vibl_open("LOG0003.VIB", VIBL_SENSOR_LIS3DSH, 16u, 100u) != VIBL_OK) {
    hil3_error_loop();
  }

  /* 4. Arm the sampling timer: TIM2 UPDATE interrupt @ exactly 1600 Hz.
        Start_IT (not plain Start): without enabling the update IRQ in NVIC
        the ISR never fires and no samples are collected at all. */
  HAL_TIM_Base_Start_IT(&htim2);

  /* 5. Main loop: commit READY ping-pong halves to SD.
        FatFs is NOT ISR-safe -> f_write/f_sync run here only, never in callbacks. */
  while (1) {
    if (lis3dsh_err != 0u) {                    /* latched sensor read failure */
      hil3_error_loop();
    }

    if (buf_ready == 1u) {                       /* buffer A: first 2048 samples */
      buf_ready = 0u;
      if (vibl_write_chunk(&pingpong_buf[0], VIBL_SAMPLES_PER_CHUNK) != VIBL_OK) {
        hil3_error_loop();
      }
    } else if (buf_ready == 2u) {               /* buffer B: next 2048 samples */
      buf_ready = 0u;
      if (vibl_write_chunk(&pingpong_buf[VIBL_SAMPLES_PER_CHUNK * 3u], VIBL_SAMPLES_PER_CHUNK) != VIBL_OK) {
        hil3_error_loop();
      }
    }

    HAL_IWDG_Refresh(&hiwdg);   /* also fed inside vibl_write_chunk() */
  }
}

/* -------------------------------------------------------------------------- */
/* TIM2 update callback (ISR context): one LIS3DSH sample per tick.          */
/* Called by HAL_TIM_IRQHandler(&htim2) from TIM2_IRQHandler:               */
/*   every 625 us -> read X/Y/Z, append [x][y][z] to pingpong_buf;          */
/*   after half (6144 halfwords) -> buf_ready = 1 (buffer A READY);         */
/*   after full (12288 halfwords) -> reset index, buf_ready = 2 (buffer B). */
/* -------------------------------------------------------------------------- */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    int16_t x, y, z;

    (void)htim;

    if (lis3dsh_read(&x, &y, &z) != 0) {
        lis3dsh_err = 1u;      /* main loop will enter the error loop */
        return;
    }

    pingpong_buf[sample_idx++] = x;
    pingpong_buf[sample_idx++] = y;
    pingpong_buf[sample_idx++] = z;

    if (sample_idx == VIBL_SAMPLES_PER_CHUNK * 3u) {          /* buffer A full */
        buf_ready = 1u;
    } else if (sample_idx == VIBL_SAMPLES_PER_CHUNK * 6u) {   /* buffer B full */
        sample_idx = 0u;
        buf_ready = 2u;
    }
}

/* -------------------------------------------------------------------------- */
static void hil3_error_loop(void)
{
    /* Blink blue LED (PB2, Active HIGH) every 100 ms and feed the watchdog. */
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
    }
}

/* -------------------------------------------------------------------------- */
/* SAFE SHUTDOWN via PC13 button — DRAFT for HIL-4 (commented out on purpose)  */
/* -------------------------------------------------------------------------- */
/* Hardware per docs/WEACT_F405_PINOUT.md §2:                                 */
/*   PC13 = User Button, wired to VDD33 through a 330 ohm resistor.          */
/*   Active HIGH: press -> pin reads GPIO_PIN_SET. There is NO external      */
/*   pull-down on the board, so the INTERNAL PULLDOWN must be enabled at     */
/*   init (already done in MX_GPIO_Init: PC13 = IT_RISING + PULLDOWN).      */
/*                                                                            */
/* Sequence on a debounced press (all work stays in main-loop context —       */
/* FatFs is NOT ISR-safe):                                                    */
/*   1) HAL_TIM_Base_Stop_IT(&htim2)  -> stop sampling, no more TIM2 IRQs     */
/*   2) vibl_close()                 -> finalize header + trailer, f_sync    */
/*   3) f_mount(NULL, "0:", 0)       -> unmount SD (flush FAT tables)         */
/*   4) Safe-Off: blink blue LED PB2 once per second + feed IWDG forever.    */
/*      Signals to the operator: "data is on disk, power may be removed".     */
/*                                                                            */
/* Debounce strategy: poll-based, 20 ms hold window (4 x 5 ms). A press       */
/* must stay HIGH for the whole window to count; any dip aborts it.         */
/* -------------------------------------------------------------------------- */

/* #define HIL3_BUTTON_DEBOUNCE_MS   20u    total debounce window              */

/* static uint8_t hil3_button_debounced(void)                               */
/* {                                                                       */
/*     if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_SET) {         */
/*         return 0u;                                                      */
/*     }                                                                   */
/*     for (uint32_t i = 0u; i < HIL3_BUTTON_DEBOUNCE_MS / 5u; i++) {       */
/*         HAL_Delay(5);                                                   */
/*         if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) != GPIO_PIN_SET) {     */
/*             return 0u;                                                  */
/*         }                                                               */
/*     }                                                                   */
/*     return 1u;                                                          */
/* }                                                                       */

/* static void hil3_safe_shutdown(void)                                    */
/* {                                                                       */
/*     HAL_TIM_Base_Stop_IT(&htim2);        step 1: stop sampling            */
/*     vibl_close();                       step 2: finalize VIBL file       */
/*     f_mount(NULL, "0:", 0);             step 3: unmount SD               */
/*     while (1) {                     step 4: Safe-Off blink + feed IWDG   */
/*         HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);                          */
/*         HAL_Delay(500);                                                 */
/*         HAL_IWDG_Refresh(&hiwdg);                                      */
/*     }                                                                   */
/* }                                                                       */

/* Main-loop integration point (inside the while(1) of main()):             */
/*   if (hil3_button_debounced()) {                                       */
/*       hil3_safe_shutdown();                                            */
/*   }                                                                     */

/**
  * @brief System Clock Configuration (same as HIL-1/HIL-2).
  *        HSE 8 MHz -> PLL: M=4, N=168, P=2 => SYSCLK = 168 MHz.
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
