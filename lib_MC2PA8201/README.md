# README #

* lib_MC2PA8201

MC2PA8201 LCD library. Depends on lib_delay.
Tested with Nokia E61, E63, N70 and E73 LCDs.
Standard LCD size is 320x240 (but supports other values). Connected through 8bit FSMC peripheral. 
Supports 12bit,16bit,18bit and 24bit color, landscape and portrait orientation.

### How do I get set up? ###

  See code example below

### Usage example ###
```C
  ... 
  MC2PA8201_Init(0,1);// FSMC write/read timings

  // only if LCD dimensions are non standard
  MC2PA8201_SetDimensions(width,height);

  // only if MADCTL parameters are non standard
  MC2PA8201_SetMADCTL_params(landscape,portrait,landscape_rev,portrait_rev);

  // Set LUT parameters for color modes that would be used
  MC2PA8201_SetLUT_params(COLOR_12BIT,sizeof(LUT12bit),LUT12bit);
  MC2PA8201_SetLUT_params(COLOR_16BIT,sizeof(LUT16bit),LUT16bit);
  MC2PA8201_SetLUT_params(COLOR_18BIT,sizeof(LUT18bit),LUT18bit);

  MC2PA8201_ColorMode(COLOR_24BIT);
  MC2PA8201_OrientationMode(ORIENTATION_PORTRAIT);
  MC2PA8201_ClearScreen(BLACK);
  ...
```
