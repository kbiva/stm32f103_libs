# README #

* lib_HD66773R

Renesas HD66773R LCD library. 
Tested with LCD from Samsung E700 mobile phone.
LCD size is 160x128. Connected through 16bit FSMC peripheral. 
Supports 16bit color, landscape and portrait orientation.
Depends on lib_delay.

### How do I get set up? ###

  See code example below

### Usage example ###
```C
  ... 
  HD66773R_Init();
  HD66773R_OrientationMode(ORIENTATION_PORTRAIT);
  HD66773R_ClearScreen(BLACK);
  ...
```
