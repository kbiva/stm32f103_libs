# README #

* lib_touch_XPT2046

XPT2046 touch controller library 
for the HY-280_262k display module with touchscreen.
Reads touch values. Calibration must be done externaly.

### How do I get set up? ###

  Uses SPI1, defined in xpt2046.h file

### Usage example ###
```C
  uint16_t x,y;
  ...
  Touch_Init();
  ...
  x  = Touch_Read(CMD_RDX);
  y  = Touch_Read(CMD_RDY);
  ...
```
