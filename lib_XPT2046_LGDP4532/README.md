# README #

* lib_XPT2046_LGDP4532 

Library for the touchpanel with XPT2046 controller
connected to the LCD with LGDP4532 controler.
Uses EXTI9_5_IRQHandler interrupt handler.
Tested with:
* 2.4" TFT LCD Display Module, Model: HY-280_262k
* HY-STM32 board with STM32F103VET6

Based on work from Andy Brown. http://andybrown.me.uk
Converted from C++ to C with some modifications.

### Usage example ###
```C
  int16_t tempx=0,tempy=0
  ...
  // initialize touchpanel
  XPT2046_TouchPanel_LGDP4532_Init();
  // initialize LCD
  LGDP4532_Init(0,2);
  LGDP4532_ColorMode(COLOR_16BIT);
  LGDP4532_OrientationMode(ORIENTATION_LANDSCAPE);
  LGDP4532_ClearScreen(BLACK);	
  LGDP4532_SetFont(FONT_8x14);
  // calibrate
  XPT2046_LGDP4532_Calibrate();
  // or set previously saved calibration values
  //XPT2046_LGDP4532_SetCalibrationValues(0.0906787589,-0.00177801482,-30.8823395,0.0000571504788,-0.0672280118,254.316681);
  ...
  while(1) {
    // wait for a click, XPT2046_clicked is set in the interrupt handler
    for(XPT2046_clicked=0;!XPT2046_clicked;);
    do {
      if(XPT2046_LGDP4532_GetCoordinates(&tempx,&tempy)) {
        // if the click is on screen, plot it. This bounds check is necessary because
        // the touch screen can and does extend past the LCD edges.
        if(tempx>=0 && tempx<=LGDP4532_GetWidth() && tempy>=0 && tempy<=LGDP4532_GetHeight()) {
          LGDP4532_SetPixel(tempx,tempy,WHITE);
        }			 
      }
    } while(!GPIO_ReadInputDataBit(XPT2046_IRQ_Port,XPT2046_IRQ_Pin));	
  }
  ...
```
