/*
 *  hy_stm32.c
 *
 *  Author: Kestutis Bivainis
 */
 
#include "stm32f10x_conf.h"
#include "hy_stm32.h"

static GPIO_TypeDef* LED_PORT[LEDn] = {LED1_PORT, LED2_PORT, LED3_PORT, LED4_PORT};
static const uint16_t LED_PIN[LEDn] = {LED1_PIN, LED2_PIN, LED3_PIN, LED4_PIN};
static const uint32_t LED_BUS[LEDn] = {LED1_BUS, LED2_BUS, LED3_BUS, LED4_BUS};

static GPIO_TypeDef* BUTTON_PORT[BUTTONn] = {BUTTON1_PORT,BUTTON2_PORT,BUTTON3_PORT,BUTTON4_PORT}; 
static const uint16_t BUTTON_PIN[BUTTONn] = {BUTTON1_PIN,BUTTON2_PIN,BUTTON3_PIN,BUTTON4_PIN}; 
static const uint32_t BUTTON_BUS[BUTTONn] = {BUTTON1_BUS,BUTTON2_BUS,BUTTON3_BUS,BUTTON4_BUS};
static const uint32_t BUTTON_AFIO_BUS[BUTTONn] = {BUTTON1_AFIO_BUS,BUTTON2_AFIO_BUS,BUTTON3_AFIO_BUS,BUTTON4_AFIO_BUS};
static const uint32_t BUTTON_PORTSOURCE[BUTTONn] = {BUTTON1_PORTSOURCE,BUTTON2_PORTSOURCE,BUTTON3_PORTSOURCE,BUTTON4_PORTSOURCE};
static const uint32_t BUTTON_PINSOURCE[BUTTONn] = {BUTTON1_PINSOURCE,BUTTON2_PINSOURCE,BUTTON3_PINSOURCE,BUTTON4_PINSOURCE};
static const uint32_t BUTTON_EXTI_LINE[BUTTONn] = {BUTTON1_EXTI_LINE,BUTTON2_EXTI_LINE,BUTTON3_EXTI_LINE,BUTTON4_EXTI_LINE};
static const uint32_t BUTTON_EXTI_IRQN[BUTTONn] = {BUTTON1_EXTI_IRQN,BUTTON2_EXTI_IRQN,BUTTON3_EXTI_IRQN,BUTTON4_EXTI_IRQN};


void BSP_LED_Init(Led_TypeDef Led) {
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd(LED_BUS[Led], ENABLE);
  
  GPIO_InitStructure.GPIO_Pin = LED_PIN[Led];
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED_PORT[Led], &GPIO_InitStructure);
  
  BSP_LED_Off(Led);
}

void BSP_LEDs_Init(void) {
  
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);
  BSP_LED_Init(LED3);
  BSP_LED_Init(LED4);
}

void BSP_LED_On(Led_TypeDef Led) {
  
  GPIO_SetBits(LED_PORT[Led],LED_PIN[Led]);
}

void BSP_LED_Off(Led_TypeDef Led) {
  
  GPIO_ResetBits(LED_PORT[Led],LED_PIN[Led]);
}

void BSP_LED_Toggle(Led_TypeDef Led) {
	
  LED_PORT[Led]->ODR ^= LED_PIN[Led];
}

void BSP_Button_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode) {

  GPIO_InitTypeDef GPIO_InitStructure;  

  RCC_APB2PeriphClockCmd(BUTTON_BUS[Button], ENABLE);
  
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Pin  = BUTTON_PIN[Button];
  GPIO_Init(BUTTON_PORT[Button], &GPIO_InitStructure);  

  if (ButtonMode == BUTTON_MODE_EXTI) {
  
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;

    //enable AFIO clock
    RCC_APB2PeriphClockCmd(BUTTON_AFIO_BUS[Button],ENABLE);

    //Connect EXTI Lines to Button Pins
    GPIO_EXTILineConfig(BUTTON_PORTSOURCE[Button], BUTTON_PINSOURCE[Button]);
  
    //select EXTI line
    EXTI_InitStructure.EXTI_Line = BUTTON_EXTI_LINE[Button];
    //select interrupt mode
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    //generate interrupt on rising edge
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    //enable EXTI line
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    //send values to registers
    EXTI_Init(&EXTI_InitStructure);
    //disable AFIO clock
    RCC_APB2PeriphClockCmd(BUTTON_AFIO_BUS[Button],DISABLE);
  
    //configure NVIC
    //select NVIC channel to configure
    NVIC_InitStructure.NVIC_IRQChannel = BUTTON_EXTI_IRQN[Button];
    //set priority to lowest
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    //set subpriority to lowest
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    //enable IRQ channel
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //update NVIC registers
    NVIC_Init(&NVIC_InitStructure);
   }
}

void BSP_Buttons_Init(ButtonMode_TypeDef ButtonMode) {

  BSP_Button_Init(BUTTON1,ButtonMode);
  BSP_Button_Init(BUTTON2,ButtonMode);
  BSP_Button_Init(BUTTON3,ButtonMode);
  BSP_Button_Init(BUTTON4,ButtonMode);
}

uint32_t BSP_Button_GetState(Button_TypeDef Button) {
  return !GPIO_ReadInputDataBit(BUTTON_PORT[Button],BUTTON_PIN[Button]);
}
