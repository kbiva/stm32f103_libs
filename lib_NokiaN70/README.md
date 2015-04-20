# README #

* lib_NokiaN70

Nokia N70 LCD library. Depends on lib_delay.
Similar to MC2PA8201 with some minor differencies:
 - resolution is 176x208 px.
 - parameters for COLOUR_SET command are different.
Tested with Nokia N70 LCD.
Connected through 8bit FSMC peripheral. 
Supports 12bit,16bit and 18bit color, landscape and portrait orientation.

### How do I get set up? ###

  See code example below

### Usage example ###
```C
  ... 
  NokiaN70_Init(0,1);
  NokiaN70_ColorMode(COLOR_18BIT);
  NokiaN70_OrientationMode(ORIENTATION_LANDSCAPE);
  NokiaN70_ClearScreen(BLACK);
  ...
```
