/**
 * @file    lis3dsh.c
 * @brief   LIS3DSH driver over SPI1 (blocking mode, ISR-safe).
 *          See include/lis3dsh.h for wiring and register values.
 */
#include "lis3dsh.h"

/* -------------------------------------------------------------------------- */
/* Peripheral handle / pin map                                               */
/* -------------------------------------------------------------------------- */
SPI_HandleTypeDef hspi1;

#define LIS3DSH_CS_PORT   GPIOA
#define LIS3DSH_CS_PIN    GPIO_PIN_4      /* PA4, active LOW (CS=0 -> selected) */

/* SPI1 clock: APB2 = 84 MHz / prescaler 16 = 5.25 MHz (within the ~5-10 MHz spec).
 * One 7-byte burst read takes ~13 us << TIM2 period 625 us. */
#define LIS3DSH_SPI_PRESCALER   SPI_BAUDRATEPRESCALER_16

/* Register map (LIS3DSH datasheet) ------------------------------------------*/
#define WHO_AM_I_ADDR     0x0Fu
#define CTRL_REG4_ADDR    0x20u
#define CTRL_REG5_ADDR    0x24u
#define CTRL_REG6_ADDR    0x25u
#define OUT_X_L_ADDR      0x28u

/* Read command with auto-increment (bit 7 = 1). */
#define READ_AUTO_INC     0x80u

/* Verified register values -------------------------------------------------*/
#define WHO_AM_I_VAL      0x3Fu   /* LIS3DSH identity (NOT 0x33 — that is LIS3DH) */
#define CTRL_REG4_VAL     0x6Eu   /* ODR=1.6 kHz (0110), X|Y|Z enabled (1110)    */
#define CTRL_REG5_VAL     0x80u   /* BDU = 1                                       */
#define CTRL_REG6_VAL     0x03u   /* Full scale ±16 g                              */

/* SPI transaction timeout (ms). A normal burst completes in ~13 us; the timeout
 * only guards a dead bus, where the IWDG (~1.5 s) is the final backstop. */
#define LIS3DSH_SPI_TIMEOUT_MS  10u

/* -------------------------------------------------------------------------- */
/* Low-level helpers                                                         */
/* -------------------------------------------------------------------------- */
static void cs_low(void)
{
    HAL_GPIO_WritePin(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN, GPIO_PIN_RESET);
}

static void cs_high(void)
{
    HAL_GPIO_WritePin(LIS3DSH_CS_PORT, LIS3DSH_CS_PIN, GPIO_PIN_SET);
}

/* Full-duplex blocking SPI transfer: `len` bytes out and in. */
static int spi_tx_rx(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    return (HAL_SPI_TransmitReceive(&hspi1, tx, rx, len, LIS3DSH_SPI_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

/* Write one register: [addr][value]; the received bytes are discarded. */
static int lis3dsh_write_reg(uint8_t addr, uint8_t value)
{
    uint8_t tx[2]      = { addr, value };
    uint8_t rx_dummy[2];

    cs_low();
    int rc = spi_tx_rx(tx, rx_dummy, 2u);
    cs_high();
    return rc;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                */
/* -------------------------------------------------------------------------- */
int lis3dsh_init(void)
{
    GPIO_InitTypeDef g;

    __HAL_RCC_GPIOA_CLK_ENABLE();   /* PA4..PA7 (MX_GPIO_Init already enables it) */

    /* CS: PA4 output push-pull, start HIGH (sensor deselected). */
    g.Pin       = LIS3DSH_CS_PIN;
    g.Mode      = GPIO_MODE_OUTPUT_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LIS3DSH_CS_PORT, &g);
    cs_high();

    /* SPI1 pins: PA5 (SCK), PA6 (MISO), PA7 (MOSI) -> AF5. */
    g.Pin       = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(LIS3DSH_CS_PORT, &g);

    /* SPI1 master, 8-bit, MSB first. Mode 3: CPOL=HIGH, CPHA=2EDGE.
     * Software CS via PA4 (no NSS pin). */
    hspi1.Instance        = SPI1;
    hspi1.Init.Mode       = SPI_MODE_MASTER;
    hspi1.Init.Direction  = SPI_DIRECTION_2LINES;
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;      /* CPOL=1 (Mode 3) */
    hspi1.Init.CLKPhase   = SPI_PHASE_2EDGE;         /* CPHA=1 (Mode 3) */
    hspi1.Init.BaudRatePrescaler = LIS3DSH_SPI_PRESCALER;
    hspi1.Init.DataSize     = SPI_DATASIZE_8BIT;
    hspi1.Init.FirstBit     = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode       = SPI_TIMODE_DISABLE;

    __HAL_RCC_SPI1_CLK_ENABLE();
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        return -1;
    }

    /* Identity check: WHO_AM_I must return 0x3F. */
    uint8_t tx[1] = { WHO_AM_I_ADDR };
    uint8_t rx[1] = { 0u };
    cs_low();
    int rc = spi_tx_rx(tx, rx, 1u);
    cs_high();
    if ((rc != 0) || (rx[0] != WHO_AM_I_VAL)) {
        return -1;
    }

    /* Sensor configuration. */
    if (lis3dsh_write_reg(CTRL_REG4_ADDR, CTRL_REG4_VAL) != 0) { return -1; }
    if (lis3dsh_write_reg(CTRL_REG5_ADDR, CTRL_REG5_VAL) != 0) { return -1; }
    if (lis3dsh_write_reg(CTRL_REG6_ADDR, CTRL_REG6_VAL) != 0) { return -1; }

    return 0;
}

int lis3dsh_read(int16_t *x, int16_t *y, int16_t *z)
{
    if ((x == NULL) || (y == NULL) || (z == NULL)) {
        return -1;
    }

    /* Burst read: addr 0xA8 = OUT_X_L | auto-increment -> 6 data bytes
     * [X_L, X_H, Y_L, Y_H, Z_L, Z_H]; rx[0] is the address echo. */
    uint8_t tx[7] = { OUT_X_L_ADDR | READ_AUTO_INC, 0u, 0u, 0u, 0u, 0u, 0u };
    uint8_t rx[7];

    cs_low();
    int rc = spi_tx_rx(tx, rx, 7u);
    cs_high();
    if (rc != 0) {
        return -1;
    }

    *x = (int16_t)(((uint16_t)rx[1] << 8) | rx[0]);
    *y = (int16_t)(((uint16_t)rx[3] << 8) | rx[2]);
    *z = (int16_t)(((uint16_t)rx[5] << 8) | rx[4]);
    return 0;
}
