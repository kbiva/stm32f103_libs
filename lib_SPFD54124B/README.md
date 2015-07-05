# README #

* lib_SPFD54124B

Orise Tech SPFD54124B LCD library. Depends on lib_delay.
Tested with Nokia 6151 LCD.
LCD size is 162x132. Connected through 8bit FSMC peripheral. 
Supports 12bit,16bit and 18bit color, landscape and portrait orientation.

### How do I get set up? ###

  See code example below

### Usage example ###
```C
  ... 
  SPFD54124B_Init(0,1);// FSMC write/read timings
  SPFD54124B_ColorMode(COLOR_18BIT);
  SPFD54124B_OrientationMode(ORIENTATION_PORTRAIT);
  SPFD54124B_ClearScreen(BLACK);
  ...
```
