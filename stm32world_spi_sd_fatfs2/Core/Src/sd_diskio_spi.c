/******************************************************************************
 *  File        : sd_diskio_spi.c
 *  Author      : ControllersTech
 *  Website     : https://controllerstech.com
 *  Date        : June 26, 2025
 *
 *  Description :
 *    This file is part of a custom STM32/Embedded tutorial series.
 *    For documentation, updates, and more examples, visit the website above.
 *
 *  Note :
 *    This code is written and maintained by ControllersTech.
 *    You are free to use and modify it for learning and development.
 ******************************************************************************/


#include "diskio.h"
#include "sd_spi.h"
#include "ff_gen_drv.h"


DSTATUS SD_disk_status(BYTE drv) {
    if (drv != 0)
        return STA_NOINIT;
    return 0;
}

DSTATUS SD_disk_initialize(BYTE drv) {
    if (drv != 0)
        return STA_NOINIT;

    return (SD_SPI_Init() == SD_OK) ? 0 : STA_NOINIT;
}

DRESULT SD_disk_read(BYTE pdrv, BYTE *buff, DWORD sector, UINT count) {
    if (pdrv != 0 || count == 0)
        return RES_PARERR;
    if (!card_initialized) return RES_NOTRDY;
    return (SD_ReadBlocks(buff, sector, count) == SD_OK) ? RES_OK : RES_ERROR;
}

DRESULT SD_disk_write(BYTE pdrv,  BYTE *buff, DWORD sector, UINT count) {
    if (pdrv || !count) return RES_PARERR;
    if (!card_initialized) return RES_NOTRDY;
    return (SD_WriteBlocks(buff, sector, count) == SD_OK) ? RES_OK : RES_ERROR;
}

DRESULT SD_disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    if (pdrv != 0)
        return RES_PARERR;

    switch (cmd) {
    case CTRL_SYNC:
        return RES_OK;
    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        return RES_OK;
    case GET_SECTOR_COUNT:
        *(DWORD *)buff = 0x10000; // Example: 32MB SD card (65536 * 512)
        return RES_OK;
    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1;
        return RES_OK;
    default:
        return RES_PARERR;
    }
}

const Diskio_drvTypeDef SD_Driver = {
    SD_disk_initialize,
    SD_disk_status,
    SD_disk_read,
#if _USE_WRITE
    SD_disk_write,
#endif
#if _USE_IOCTL
    SD_disk_ioctl,
#endif
};
