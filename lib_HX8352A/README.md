# README #

* lib_HX8352A

HX8352A LCD library. Depends on lib_delay.
Tested with LCD from LG KF700 mobile phone.
LCD size is 480x240. Connected through 16bit FSMC peripheral. 
Supports 16bit and 18bit color, landscape and portrait orientation.

### How do I get set up? ###

  When calling HX8352A_Init() specify color mode.

### Usage example ###

  ... 
  GPIO_Configuration();
  FSMC_LCD_Init();	
  HX8352A_Init(COLOR_18BIT);
  HX8352A_OrientationMode(ORIENTATION_PORTRAIT);
  ...
  HX8352A_ClearScreen(BLACK);
  ...