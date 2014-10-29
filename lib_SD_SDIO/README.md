# README #

* lib_SD_SDIO

SD SDIO low level API.
Ported source code from STM3210E-EVAL board.

### How do I get set up? ###

SDIO and DMA2 Channel4_5 interrupt handlers must be configured in the program.

### Usage example ###
 
  The library is used together with lib_FatFs and lib_SD_FatFs,
  to access FAT file system on SD card.

  It also can be used without FatFs to accsess SD card information:

```C
  ...
  void SDIO_IRQHandler(void) {
    SD_ProcessIRQSrc();
  }
  ...
  void DMA2_Channel4_5_IRQHandler(void) {
    SD_ProcessDMAIRQ();
  }
  ...
  SD_Error Status;
  SD_CardInfo SDCardInfo;
  ...
  SD_IRQ_Conf();
  Status = SD_Init();
  if(Status==SD_OK) {
    Status = SD_GetCardInfo(&SDCardInfo);
  }
```