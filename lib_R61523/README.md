# README #

* lib_R61523

R61523 LCD library. Depends on lib_delay.
Tested with LCD from Sony Ericsson Vivaz U5 mobile phone.
LCD size is 640x360. Connected through 16bit FSMC peripheral. 
Supports 16bit,18bit and 24bit color, landscape and portrait orientation.

### How do I get set up? ###

  See code example below

### Usage example ###
```C
  ... 
  R61523_Init();
  R61523_ColorMode(COLOR_24BIT);
  R61523_OrientationMode(ORIENTATION_PORTRAIT);
  R61523_ClearScreen(BLACK);
  ...
```
