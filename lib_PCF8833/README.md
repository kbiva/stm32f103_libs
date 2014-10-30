# README #

* lib_PCF8833

PCF8833 LCD library. Depends on lib_delay.
Tested with LCDs from Nokia 3100,6100,6020,6030 mobile phones.
LCD size is 132x132, visible area is 130x130 
(first and last pixel rows and first and last pixel columns are invisible).
Connected through SPI or bitbang. 
Supports 8bit,12bit and 16bit color, landscape and portrait orientation.
Included fonts sizes are 6x8, 8x8 and 8x16.

Summary of LCD diferencies

| LCD | Type | Color format | Colormap for 8bit color | Colormap for 12bit color | Display inversion |
|-----|------|--------------|-----------------------------|---------------------------|--------------------|
| 3100 | CSTN | RGB | 20 bytes | not needed | ? |
| 6100 | CSTN | BGR | 20 bytes | not needed | doesn't work
| 6020 | TFT | RGB | 48 bytes | 48 bytes | works |
| 6030 | CSTN | RGB | 20 bytes | not needed | ? |

Also Nokia 6020 LCD has slight differencies in orientation handling.

### How do I get set up? ###

Setup for Nokia 6100 LCD:
```C
   PCF8833_Init(ACCESS_SPI9BITS);
   //PCF8833_Init(ACCESS_BITBANG);
   // Color mode setup
   PCF8833_ColorMode(COLOR_8BIT);
   //PCF8833_ColorMode(COLOR_12BIT);
   //PCF8833_ColorMode(COLOR_16BIT);
   PCF8833_SetRGB(MODE_BGR);
   PCF8833_SetupColor(RGB8ColorMap,20);// when using 8bit color mode
   // LCD contrast setup
   PCF8833_SetContrast(...);
   // LCD orientation setup
   PCF8833_SetOrientation(ORIENTATION_LANDSCAPE,0);
   //PCF8833_SetOrientation(ORIENTATION_LANDSCAPE_REV,0);
   //PCF8833_SetOrientation(ORIENTATION_PORTRAIT,0);
   //PCF8833_SetOrientation(ORIENTATION_PORTRAIT_REV,0);
```

Example setup for Nokia 6020 LCD:
```C
   PCF8833_Init(ACCESS_SPI9BITS);
   //PCF8833_Init(ACCESS_BITBANG);
   // Color mode setup
   //PCF8833_ColorMode(COLOR_8BIT);
   PCF8833_ColorMode(COLOR_12BIT);
   //PCF8833_ColorMode(COLOR_16BIT);
   //PCF8833_SetupColor(RGB8ColorMap_Nokia6020,48);// when using 8bit color mode
   PCF8833_SetupColor(RGB12ColorMap_Nokia6020,48);// when using 12bit color mode
   // LCD contrast setup
   PCF8833_SetContrast(...);
   // LCD orientation setup
   PCF8833_SetOrientation(ORIENTATION_LANDSCAPE1,0);
   //PCF8833_SetOrientation(ORIENTATION_LANDSCAPE_REV1,0);
   //PCF8833_SetOrientation(ORIENTATION_PORTRAIT1,1);
   //PCF8833_SetOrientation(ORIENTATION_PORTRAIT_REV1,1);
```

### Usage example ###

After initialization

```C
  PCF8833_SetFont(FONT_6x8);
  PCF8833_PutStr("Hello world,10,10);
```