# README #

* lib_S6B33BG

Samsung S6B33BG LCD library. 
Tested with LCD from Samsung GT-E1050 mobile phone.
LCD size is 128x128. Connected through 8bit FSMC peripheral. 
Supports 12bit and 16bit color, landscape and portrait orientation.
Depends on lib_delay.

### How do I get set up? ###

  See code example below

### Usage example ###
```C
  ... 
  S6B33BG_Init();
  S6B33BG_ColorMode(COLOR_16BIT);
  S6B33BG_OrientationMode(ORIENTATION_PORTRAIT);
  S6B33BG_ClearScreen(BLACK);
  ...
```
