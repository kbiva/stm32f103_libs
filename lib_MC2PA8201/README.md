# README #

* lib_MC2PA8201

MC2PA8201 LCD library. Depends on lib_delay.
Tested with Nokia E63 and E73 LCDs.
LCD size is 320x240. Connected through 8bit FSMC peripheral. 
Supports 12bit,16bit,18bit and 24bit color, landscape and portrait orientation.

### How do I get set up? ###

  See code example below

### Usage example ###
```C
  ... 
  MC2PA8201_Init(0,1);// FSMC write/read timings
  MC2PA8201_ColorMode(COLOR_24BIT);
  MC2PA8201_OrientationMode(ORIENTATION_PORTRAIT);
  MC2PA8201_ClearScreen(BLACK);
  ...
```
