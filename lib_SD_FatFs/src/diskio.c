/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"    /* FatFs lower layer API */
#include "stm3210e_eval_sdio_sd.h"
#include "ffconf.h"
#include "rtc.h"
#include <string.h>
#include "stm32f10x_conf.h"

extern SD_CardInfo SDCardInfo;

DSTATUS disk_initialize (
  BYTE pdrv        /* Physical drive nmuber (0..) */
)
{
  SD_Error Status;

  Status = SD_Init();
  if(Status == SD_OK)
    return 0;
  else
    return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Get Disk Status                                                       */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
  BYTE pdrv    /* Physical drive number (0..) */
)
{
  return 0;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
  BYTE pdrv,    /* Physical drive number (0..) */
  BYTE *buff,    /* Data buffer to store read data */
  DWORD sector,  /* Sector address (LBA) */
  BYTE count    /* Number of sectors to read (1..128) */
)
{
  SD_Error status=SD_OK;

  if ((DWORD)buff & 3) { // DMA Alignment failure, do single up to aligned buffer

    DRESULT res = RES_OK;
    DWORD scratch[_MAX_SS / 4]; // Alignment assured, you'll need a sufficiently big stack

    while(count--) {
      res = disk_read(pdrv, (void *)scratch, sector++, 1);
      if (res != RES_OK)
         break;

      memcpy(buff, scratch, _MAX_SS);
      buff += _MAX_SS;
    }
    return(res);
  }

  status = SD_ReadMultiBlocks((uint8_t *)buff, sector << 9 ,_MAX_SS,count);
  if ( status != SD_OK )
    return RES_ERROR;

  status =SD_WaitReadOperation();
  if ( status != SD_OK )
    return RES_ERROR;

  while(SD_GetStatus() != SD_TRANSFER_OK){};

  return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if _USE_WRITE
DRESULT disk_write (
  BYTE pdrv,      /* Physical drive number (0..) */
  const BYTE *buff,  /* Data to be written */
  DWORD sector,    /* Sector address (LBA) */
  BYTE count      /* Number of sectors to write (1..128) */
)
{
  SD_Error status = SD_OK;

  status = SD_WriteMultiBlocks((uint8_t *)buff, sector << 9 ,_MAX_SS,count);
  if ( status != SD_OK )
    return RES_ERROR;

  status = SD_WaitWriteOperation();
  if ( status != SD_OK )
    return RES_ERROR;

  while(SD_GetStatus() != SD_TRANSFER_OK){};

  return RES_OK;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
  BYTE pdrv,    /* Physical drive number (0..) */
  BYTE cmd,    /* Control code */
  void *buff    /* Buffer to send/receive control data */
)
{

  DRESULT res;

  switch(cmd)
  {
    case CTRL_SYNC:
      while(SDIO_GetResponse(SDIO_RESP1)==0);// SD_WaitReady
      res = RES_OK;
      break;
    case GET_SECTOR_SIZE:
      *(WORD*)buff = 512;
      res = RES_OK;
      break;
    case GET_SECTOR_COUNT:
        if((SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) || (SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0))
            *(DWORD*)buff = SDCardInfo.CardCapacity >> 9;
        else if(SDCardInfo.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
            *(DWORD*)buff = (SDCardInfo.SD_csd.DeviceSize+1)*1024;
        //else;////SD_GetCapacity();
        res = RES_OK;
        break;
    case GET_BLOCK_SIZE:
        *(WORD*)buff = SDCardInfo.CardBlockSize;
        res = RES_OK;
        break;
    case CTRL_ERASE_SECTOR:
    case CTRL_POWER:
    case CTRL_LOCK:
    case CTRL_EJECT:
    case CTRL_FORMAT:
      res = RES_PARERR;
      break;
    case MMC_GET_TYPE:
      *(BYTE*)buff = SDCardInfo.CardType;
      res = RES_OK;
      break;
    case MMC_GET_CSD:
    case MMC_GET_CID:
    case MMC_GET_OCR:
    case MMC_GET_SDSTAT:
        res = RES_PARERR;
    default:
        res = RES_PARERR;
        break;
  }
  return res;
}
#endif

DWORD get_fattime(void){

  RTC_t t;
  DWORD res;

  RTC_gettime(&t);

  res=(((DWORD)t.year - 1980) << 25)
      | ((DWORD)t.month << 21)
      | ((DWORD)t.mday << 16)
      | (WORD)(t.hour << 11)
      | (WORD)(t.min << 5)
      | (WORD)(t.sec >> 1);

  return res;
}
