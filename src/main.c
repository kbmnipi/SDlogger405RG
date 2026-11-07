/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main_hil1.c
  * @brief          : HIL-1 test for SD Logger V2 (WeAct STM32F405RG, FatFs R0.12c).
  *
  * Purpose: validate the FatFs + sd_writer (VIBL) chain WITHOUT ADC/DAC/TIM2.
  * Writes one file "LOG0001.VIB" with a correct header, 10 chunks of fake
  * triangle-wave data and a finalized trailer, then blinks the blue LED (PB2).
  *
  * LED semantics (docs/WEACT_F405_PINOUT.md: PB2 = LED Blue, Active HIGH):
  *   - mount/open/write/close failure -> LED ON + IWDG-fed infinite loop
  *   - success                        -> blink every 100 ms
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

/* Private variables ---------------------------------------------------------*/
FATFS fs;

/* Fake data buffer: 2048 samples x 3 axes (int16) = 12 KiB, static. */
static int16_t test_buf[VIBL_SAMPLES_PER_CHUNK * 3u];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration --------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals (no ADC/DAC/TIM2 for HIL-1). */
  MX_GPIO_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();

  /* 1. Mount the SD card (format=1: create FAT if the card is unformatted,
        which is convenient for the HIL bench). */
  if (f_mount(&fs, "0:", 1) != FR_OK) {
    /* No filesystem -> blink blue LED forever. */
    while (1) {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
      HAL_Delay(100);
      HAL_IWDG_Refresh(&hiwdg);
    }
  }

  /* 2. Открытие файла VIBL с выводом кода ошибки */
  int open_status = vibl_open("LOG0001.VIB", VIBL_SENSOR_HIL, 16u, 100u);
  if (open_status != VIBL_OK) {
    while (1) {
      // 3 = OPEN, 4 = WRITE, 5 = SYNC
      int blinks = (open_status < 0) ? -open_status : 10;
      for (int i = 0; i < blinks; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_Delay(200);
      }
      HAL_Delay(1000); // Пауза между сериями
      HAL_IWDG_Refresh(&hiwdg);
    }
  }

  /* 3. Write 10 chunks of fake data (triangle wave on X/Y, zero on Z). */
  for (uint32_t chunk = 0u; chunk < 10u; chunk++) {
    /* Fill the buffer: sample s occupies test_buf[3s..3s+2] (x, y, z). */
    for (uint32_t s = 0u; s < VIBL_SAMPLES_PER_CHUNK; s++) {
      test_buf[3u * s + 0u] = (int16_t)((s % 200u) - 100u);        /* X: triangle */
      test_buf[3u * s + 1u] = (int16_t)(((s % 100u) * 2u) - 100u);  /* Y: triangle */
      test_buf[3u * s + 2u] = 0;                                   /* Z: zero     */
    }

    if (vibl_write_chunk(test_buf, VIBL_SAMPLES_PER_CHUNK) != VIBL_OK) {
      /* Chunk write failed -> LED ON and stop. */
      while (1) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_IWDG_Refresh(&hiwdg);
      }
    }

    HAL_IWDG_Refresh(&hiwdg);   /* keep the watchdog fed between chunks */
    HAL_Delay(500);             /* pacing: ~1 chunk per 0.7 s worst case */
  }

  /* 4. Finalize: header record_count + trailer, f_sync, close. */
  if (vibl_close() != VIBL_OK) {
    while (1) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
      HAL_IWDG_Refresh(&hiwdg);
    }
  }

  /* 5. Success: blink blue LED every 100 ms forever. */
  while (1) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
    HAL_Delay(100);
    HAL_IWDG_Refresh(&hiwdg);
  }
}

/**
  * @brief System Clock Configuration (copied from main.c).
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
