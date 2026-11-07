/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_hil2.c
  * @brief          : HIL-2 test for SD Logger V2 (WeAct STM32F405RG, FatFs R0.12c).
  *
  * Purpose: end-to-end signal chain validation WITHOUT the LIS3DSH:
  *   DAC_CH2 (PA5) plays a 10 Hz sine from a DMA LUT, loopback jumpers
  *   PA5 -> PA0/PA1/PA2 feed ADC1 (3 channels, 12-bit), which is sampled at
  *   exactly 1600 Hz by TIM2 TRGO. Results stream into a ping-pong buffer via
  *   DMA2_Stream0 (circular); the main loop commits READY halves to SD as VIBL
  *   chunks through vibl_write_chunk().
  *
  * Pin map per docs/WEACT_F405_PINOUT.md:
  *   PA5 = DAC_OUT2 (HIL conflict with SPI1_SCK flagged in pinout doc, OK here)
  *   PA0 / PA1 / PA2 = ADC1_IN0..IN2
  *   PB2 = LED Blue, Active HIGH
  *
  * Timing budget:
  *   TIM2: 84 MHz / ((124+1)*(419+1)) = exactly 1600 Hz -> period 625 us.
  *   One ping-pong half = 2048 samples = 640 ms of signal.
  *   Chunk commit: f_write(12 KiB) ~3..10 ms + f_sync <= ~200 ms worst case
  *   (see sd_writer.c comment) -> CPU busy <= ~250 ms per 1.28 s (~20%).
  *   DMA keeps filling the other half during f_sync: no data loss by design.
  *
  * LED semantics (PB2, Active HIGH): any failure -> blink every 100 ms +
  * IWDG-fed infinite loop.
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
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "dma.h"

#include <math.h>   /* sinf() for LUT fill */

/* Private variables ---------------------------------------------------------*/
FATFS fs;

/* Ping-pong ADC buffer: 2 x 2048 samples x 3 axes (int16) = 12288 halfwords
 * = 24 KiB, static. Layout per scan: [x][y][z] interleaved — exactly the VIBL
 * on-disk record layout, so chunks are written zero-copy. */
static int16_t pingpong_buf[VIBL_SAMPLES_PER_CHUNK * 2u * 3u];

/* DMA half/complete flags: set in ADC DMA callbacks (ISR context), consumed by
 * the main loop ONLY. Values: 0 = none, 1 = buffer A READY, 2 = buffer B READY. */
static volatile uint8_t buf_ready = 0u;

/* Sine LUT for DAC_CH2: 160 samples per period @ 1600 Hz -> exactly 10 Hz.
 * Filled at startup with sinf() (real sine, not a triangle): mid-scale 2048,
 * amplitude 1500 to keep the direct DAC->ADC loopback away from the rails. */
#define SINE_LUT_SIZE 160u
static uint16_t sine_lut[SINE_LUT_SIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void hil2_error_loop(void);
static void sine_lut_fill(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration --------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals (HIL-2 adds ADC/DAC/TIM2 vs HIL-1). */
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC_Init();
  MX_TIM2_Init();

  /* Build the DAC sine table before arming any DMA. */
  sine_lut_fill();

  /* 1. Mount the SD card (format=1: create FAT if the card is unformatted). */
  if (f_mount(&fs, "0:", 1) != FR_OK) {
    hil2_error_loop();
  }

  /* 2. Open the VIBL file: HIL sensor id, 16 g range, gain = 100 LSB/g. */
  if (vibl_open("LOG0002.VIB", VIBL_SENSOR_HIL, 16u, 100u) != VIBL_OK) {
    hil2_error_loop();
  }

  /* 3. Arm the chain — STRICT ORDER:
        a) DAC DMA first (circular LUT playback),
        b) ADC DMA second (ping-pong arming; no conversion yet, external trig),
        c) TIM2 last — its TRGO edges start BOTH DAC output and ADC scans. */
  if (HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, (uint32_t *)sine_lut, SINE_LUT_SIZE, DAC_ALIGN_12B_R) != HAL_OK) {
    hil2_error_loop();
  }
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)pingpong_buf, VIBL_SAMPLES_PER_CHUNK * 2u * 3u) != HAL_OK) {
    hil2_error_loop();
  }
  HAL_TIM_Base_Start(&htim2);

  /* 4. Main loop: commit READY ping-pong halves to SD.
        FatFs is NOT ISR-safe -> f_write/f_sync run here only, never in callbacks. */
  while (1) {
    if (buf_ready == 1u) {                       /* buffer A: first 2048 samples */
      buf_ready = 0u;
      if (vibl_write_chunk(&pingpong_buf[0], VIBL_SAMPLES_PER_CHUNK) != VIBL_OK) {
        hil2_error_loop();
      }
    } else if (buf_ready == 2u) {               /* buffer B: next 2048 samples */
      buf_ready = 0u;
      if (vibl_write_chunk(&pingpong_buf[VIBL_SAMPLES_PER_CHUNK * 3u], VIBL_SAMPLES_PER_CHUNK) != VIBL_OK) {
        hil2_error_loop();
      }
    }

    HAL_IWDG_Refresh(&hiwdg);   /* also fed inside vibl_write_chunk() */
  }
}

/* -------------------------------------------------------------------------- */
/* ADC DMA callbacks (ISR context): flag setting ONLY.                        */
/* Called by HAL_DMA_IRQHandler(&hdma_adc1) from DMA2_Stream0_IRQHandler:    */
/*   ConvHalfCplt -> after first 6144 halfwords (buffer A full, t ~ 640 ms)   */
/*   ConvCplt     -> after 12288 halfwords (buffer B full; circular DMA       */
/*                   auto-restarts at pingpong_buf[0])                       */
/* -------------------------------------------------------------------------- */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    buf_ready = 1u;   /* buffer A READY — main loop will commit it */
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    (void)hadc;
    buf_ready = 2u;   /* buffer B READY — main loop will commit it */
}

/* -------------------------------------------------------------------------- */
static void hil2_error_loop(void)
{
    /* Blink blue LED (PB2, Active HIGH) every 100 ms and feed the watchdog. */
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
        HAL_Delay(100);
        HAL_IWDG_Refresh(&hiwdg);
    }
}

static void sine_lut_fill(void)
{
    const float step = 6.283185307179586f / (float)SINE_LUT_SIZE;   /* 2*pi/160 */
    for (uint32_t i = 0u; i < SINE_LUT_SIZE; i++) {
        sine_lut[i] = (uint16_t)(2048.0f + 1500.0f * sinf(step * (float)i));
    }
}

/**
  * @brief System Clock Configuration (same as HIL-1).
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
