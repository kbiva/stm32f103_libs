# README #

* lib_R63400

Renesas R63400 LCD library. Depends on lib_delay.
Tested with Sony Ericsson K800i LCD.
Standard LCD size is 320x240. Connected through 8bit FSMC peripheral. 
Supports 16bit and 18bit color, landscape and portrait orientation.

### How do I get set up? ###

  See code example below

### Usage example ###
```C
  ... 
  R63400_Init(0,2);// FSMC write/read timings

  R63400_ColorMode(COLOR_18BIT);
  R63400_OrientationMode(ORIENTATION_PORTRAIT);
  R63400_ClearScreen(BLACK);
  ...
```
