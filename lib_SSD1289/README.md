# README #

* lib_SSD1289

SSD1289 LCD library. Depends on lib_delay.
LCD size is 320x240. Connected through 16bit FSMC peripheral. 
Supports 16bit and 18bit color, landscape and portrait orientation.

### How do I get set up? ###

  See code example below

### Usage example ###
```C
  ... 
  SSD1289_Init();
  SSD1289_ColorMode(COLOR_16BIT);
  SSD1289_OrientationMode(ORIENTATION_PORTRAIT);
  SSD1289_ClearScreen(BLACK);
  ...
```