# README #

* lib_ILI9328

ILI9328 LCD library. Depends on lib_delay.
LCD size is 320x240. Connected through 8bit FSMC peripheral. 
Supports 16bit and 18bit color, landscape and portrait orientation.

### How do I get set up? ###

  See code example below

### Usage example ###
```C
  ... 
  ILI9328_Init();
  ILI9328_ColorMode(COLOR_18BIT);
  ILI9328_OrientationMode(ORIENTATION_LANDSCAPE);
  ILI9328_ClearScreen(BLACK);
  ...
```