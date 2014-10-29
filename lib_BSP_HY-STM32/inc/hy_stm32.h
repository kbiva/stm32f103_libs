/*
 *  hy_stm32.h
 *
 *  Author: Kestutis Bivainis
 */

#ifndef _HY_STM32_H_
#define _HY_STM32_H_

typedef enum 
{
  LED1 = 0,
  LED2 = 1,
  LED3 = 2,
  LED4 = 3
}Led_TypeDef;

typedef enum 
{  
  BUTTON1 = 0,
  BUTTON2 = 1,
  BUTTON3 = 2,
  BUTTON4 = 3,
}Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
}ButtonMode_TypeDef;     

#define LEDn 4

#define LED1_PIN  GPIO_Pin_6
#define LED1_PORT GPIOC
#define LED1_BUS  RCC_APB2Periph_GPIOC

#define LED2_PIN  GPIO_Pin_7
#define LED2_PORT GPIOC
#define LED2_BUS  RCC_APB2Periph_GPIOC

#define LED3_PIN  GPIO_Pin_13
#define LED3_PORT GPIOD
#define LED3_BUS  RCC_APB2Periph_GPIOD

#define LED4_PIN  GPIO_Pin_6
#define LED4_PORT GPIOD
#define LED4_BUS  RCC_APB2Periph_GPIOD

#define BUTTONn 4

#define BUTTON1_PIN        GPIO_Pin_5
#define BUTTON1_PORT       GPIOE
#define BUTTON1_BUS        RCC_APB2Periph_GPIOE
#define BUTTON1_AFIO_BUS   RCC_APB2Periph_AFIO 
#define BUTTON1_PORTSOURCE GPIO_PortSourceGPIOE
#define BUTTON1_PINSOURCE  GPIO_PinSource5
#define BUTTON1_EXTI_LINE  EXTI_Line5
#define BUTTON1_EXTI_IRQN  EXTI9_5_IRQn

#define BUTTON2_PIN        GPIO_Pin_4
#define BUTTON2_PORT       GPIOE
#define BUTTON2_BUS        RCC_APB2Periph_GPIOE
#define BUTTON2_AFIO_BUS   RCC_APB2Periph_AFIO 
#define BUTTON2_PORTSOURCE GPIO_PortSourceGPIOE
#define BUTTON2_PINSOURCE  GPIO_PinSource4
#define BUTTON2_EXTI_LINE  EXTI_Line4
#define BUTTON2_EXTI_IRQN  EXTI4_IRQn

#define BUTTON3_PIN        GPIO_Pin_3
#define BUTTON3_PORT       GPIOE
#define BUTTON3_BUS        RCC_APB2Periph_GPIOE
#define BUTTON3_AFIO_BUS   RCC_APB2Periph_AFIO 
#define BUTTON3_PORTSOURCE GPIO_PortSourceGPIOE
#define BUTTON3_PINSOURCE  GPIO_PinSource3
#define BUTTON3_EXTI_LINE  EXTI_Line3
#define BUTTON3_EXTI_IRQN  EXTI3_IRQn

#define BUTTON4_PIN        GPIO_Pin_2
#define BUTTON4_PORT       GPIOE
#define BUTTON4_BUS        RCC_APB2Periph_GPIOE
#define BUTTON4_AFIO_BUS   RCC_APB2Periph_AFIO 
#define BUTTON4_PORTSOURCE GPIO_PortSourceGPIOE
#define BUTTON4_PINSOURCE  GPIO_PinSource2
#define BUTTON4_EXTI_LINE  EXTI_Line2
#define BUTTON4_EXTI_IRQN  EXTI2_IRQn


void BSP_LED_Init(Led_TypeDef Led);
void BSP_LEDs_Init(void);
void BSP_LED_On(Led_TypeDef Led);
void BSP_LED_Off(Led_TypeDef Led);
void BSP_LED_Toggle(Led_TypeDef Led);

void BSP_Button_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void BSP_Buttons_Init(ButtonMode_TypeDef ButtonMode);
uint32_t BSP_Button_GetState(Button_TypeDef Button);

#endif
