/**
  ******************************************************************************
  * @file    stm3210e_eval.h
  * @author  MCD Application Team
  * @version V5.1.0
  * @date    18-January-2013
  * @brief   This file contains definitions for STM3210E_EVAL's Leds, push-buttons
  *          COM ports, sFLASH (on SPI) and Temperature Sensor LM75 (on I2C)
  *          hardware resources.  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

#include <stdint.h>

/**
  * @brief  SD FLASH SDIO Interface
  */

#define SDIO_FIFO_ADDRESS                ((uint32_t)0x40018080)
/**
  * @brief  SDIO Intialization Frequency (400KHz max)
  */
// 72Mhz
#define SDIO_INIT_CLK_DIV                ((uint8_t)0xB2)
// 128Mhz
//#define SDIO_INIT_CLK_DIV                ((uint8_t)0xFD)
/**
  * @brief  SDIO Data Transfer Frequency (25MHz max)
  */
// 72Mhz
#define SDIO_TRANSFER_CLK_DIV            ((uint8_t)0x01)
// 128Mhz
//#define SDIO_TRANSFER_CLK_DIV            ((uint8_t)0x04)

#define SD_SDIO_DMA                      DMA2
#define SD_SDIO_DMA_CLK                  RCC_AHBPeriph_DMA2
#define SD_SDIO_DMA_CHANNEL              DMA2_Channel4
#define SD_SDIO_DMA_FLAG_TC              DMA2_FLAG_TC4
#define SD_SDIO_DMA_FLAG_TE              DMA2_FLAG_TE4
#define SD_SDIO_DMA_FLAG_HT              DMA2_FLAG_HT4
#define SD_SDIO_DMA_FLAG_GL              DMA2_FLAG_GL4
#define SD_SDIO_DMA_IRQn                 DMA2_Channel4_5_IRQn
#define SD_SDIO_DMA_IRQHANDLER           DMA2_Channel4_5_IRQHandler

void SD_LowLevel_Init(void);
void SD_LowLevel_DeInit(void);
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize);
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize);
uint32_t SD_DMAEndOfTransferStatus(void);
void SD_IRQ_Conf(void);
