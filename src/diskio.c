/**
  ******************************************************************************
  * @file    diskio.c
  * @brief   FatFs low-level disk I/O driver (HAL_SD backend).
  ******************************************************************************
  */

#include "ff.h"      /* FatFs public definitions */
#include "diskio.h"  /* FatFs disk I/O definitions */
#include "sdio.h"    /* extern SD_HandleTypeDef hsd;   */
#include "iwdg.h"    /* extern IWDG_HandleTypeDef hiwdg; */

/*---------------------------------------------------------------------------*/
/* FatFs disk status                                                         */
/*---------------------------------------------------------------------------*/
DSTATUS disk_status(BYTE pdrv)
{
    (void)pdrv; /* Only one volume is supported. */

    /* Card detect on PA8: HIGH = card absent, LOW = card present. */
    if ((GPIOA->IDR & GPIO_PIN_8) != 0U) {
        return STA_NODISK;
    }
    return 0; /* Drive ready (card present). */
}

/*---------------------------------------------------------------------------*/
/* FatFs disk initialization                                                 */
/*---------------------------------------------------------------------------*/
DSTATUS disk_initialize(BYTE pdrv)
{
    (void)pdrv; /* Only one volume is supported. */

    if (HAL_SD_Init(&hsd) != HAL_OK) {
        return STA_NOINIT;
    }

            /* HIL-1 fix: 4-bit wide bus call REMOVED. sdio.c already configures the
       stable 1-bit mode (SDIO_BUS_WIDE_1B, ClockDiv = 16). Calling
       HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) here would
       override it and reintroduce instability. Only HAL_SD_Init is kept. */



    return 0; /* Drive ready and initialized. */
}

/*---------------------------------------------------------------------------*/
/* FatFs sector read                                                         */
/*---------------------------------------------------------------------------*/
DRESULT disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
    (void)pdrv;

    /* HAL expects uint8_t*, cast the BYTE* */
    if (HAL_SD_ReadBlocks(&hsd, (uint8_t *)buff, sector, count, 1000U) != HAL_OK) {
        return RES_ERROR;
    }

    HAL_IWDG_Refresh(&hiwdg);
    return RES_OK;
}

/*---------------------------------------------------------------------------*/
/* FatFs sector write                                                        */
/*---------------------------------------------------------------------------*/
DRESULT disk_write(BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
    (void)pdrv;

    if (HAL_SD_WriteBlocks(&hsd, (uint8_t *)buff, sector, count, 1000U) != HAL_OK) {
        return RES_ERROR;
    }

    // Даем карте немного времени выйти из состояния BUSY
    HAL_Delay(10); 

    HAL_IWDG_Refresh(&hiwdg);
    return RES_OK;
}

/*---------------------------------------------------------------------------*/
/* FatFs disk control (ioctl)                                                */
/*---------------------------------------------------------------------------*/
DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
    (void)pdrv;

    switch (cmd) {
        case CTRL_SYNC:
            /* HAL_SD transfers are synchronous; nothing to flush. */
            return RES_OK;

        case GET_SECTOR_COUNT:
            /* hsd.SdCard.LogBlockNbr is defined in HAL for F4 */
            *(DWORD *)buff = hsd.SdCard.LogBlockNbr;
            return RES_OK;

        case GET_SECTOR_SIZE:
            *(WORD *)buff = 512U;
            return RES_OK;

        case GET_BLOCK_SIZE:
            *(DWORD *)buff = 1U;
            return RES_OK;

        default:
            return RES_PARERR;
    }
}