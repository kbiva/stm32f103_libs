# README #

* lib_BSP_HY-STM32

HY-STM32 board support library for onboard LEDs and buttons.

### How do I get set up? ###

If using BUTTON_MODE_EXTI, interrupt handler for EXTI line must be configured in the program.

### Usage example ###
```C
  void EXTI2_IRQHandler(void) {
    if(EXTI_GetITStatus(EXTI_Line2) != RESET) {
      // BUTTON4 is pressed
    }
    EXTI_ClearITPendingBit(EXTI_Line2);
  }
  ...

  uint32_t button_state;

  BSP_LED_Init(LED1);
  ...
  BSP_LED_On(LED1);
  ...
  BSP_Button_Init(BUTTON3,BUTTON_MODE_GPIO);
  BSP_Button_Init(BUTTON4,BUTTON_MODE_EXTI);
  ...
  button_state=BSP_Button_GetState(BUTTON3);
  ...
```

