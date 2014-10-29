# README #

* lib_LGDP4532

LGDP4532 LCD library for the HY-280_262k display module.
LCD size is 320x240. Connected through 16bit FSMC peripheral. 
Supports 16bit and 18bit color, landscape and portrait orientation.
Depends on lib_delay.

### How do I get set up? ###

  See code example below

### Usage example ###
```C
  ... 
  LGDP4532_Init();
  LGDP4532_ColorMode(COLOR_16BIT);
  LGDP4532_OrientationMode(ORIENTATION_PORTRAIT);
  LGDP4532_ClearScreen(BLACK);
  ...
```
