/**
 * @file    lis3dsh.h
 * @brief   LIS3DSH accelerometer driver over SPI1 (WeAct STM32F405RG).
 *
 * Hardware wiring (docs/WEACT_F405_PINOUT.md, section 6; jumpered to the
 * Discovery board where the LIS3DSH sits):
 *   PA4 = CS  (software driven, active LOW) -> Discovery PE3
 *   PA5 = SPI1_SCK (AF5)                    -> Discovery PA5
 *   PA6 = SPI1_MISO (AF5)                   -> Discovery PA6
 *   PA7 = SPI1_MOSI (AF5)                   -> Discovery PA7
 *
 * SPI Mode 3 (CPOL=1, CPHA=1), ~5.25 MHz (APB2 84 MHz / prescaler 16).
 * Register values per LIS3DSH datasheet:
 *   WHO_AM_I (0x0F) = 0x3F
 *   CTRL_REG4 (0x20) = 0x6E : ODR 1.6 kHz + X/Y/Z enabled
 *   CTRL_REG5 (0x24) = 0x80 : BDU=1
 *   CTRL_REG6 (0x25) = 0x03 : full scale ±16 g
 */
#ifndef LIS3DSH_H
#define LIS3DSH_H

#include "main.h"   /* stm32f4xx_hal.h -> SPI_HandleTypeDef, GPIO_*, HAL_SPI_* */

#ifdef __cplusplus
extern "C" {
#endif

extern SPI_HandleTypeDef hspi1;

/**
 * Configure SPI1 + the CS pin and initialize the LIS3DSH.
 * @retval 0 on success (WHO_AM_I == 0x3F and all register writes OK), -1 otherwise.
 */
int lis3dsh_init(void);

/**
 * Read one X/Y/Z sample via a 6-byte burst read with auto-increment
 * (address 0xA8 = OUT_X_L | 0x80). Blocking SPI transfer, ISR-safe.
 * @retval 0 on success, -1 on SPI failure or NULL argument.
 */
int lis3dsh_read(int16_t *x, int16_t *y, int16_t *z);

#ifdef __cplusplus
}
#endif

#endif /* LIS3DSH_H */
