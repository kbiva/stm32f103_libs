# README #

* lib_DS18B20

DS18D20 digital temperature sensor library. Depends on lib_1-Wire.
Supports multiple DS18B20s on the one 1-Wire line. 

### How do I get set up? ###

Configuration is done in the header file "1-Wire.h"
Adjustable parameters:

* 1-Wire Pin that is connected to DS18B20 digital thermometers.

### Usage example ###
```C 
  uint8_t cnt,j,res;
  unsigned char rom_sn[10][DS18B20_SERIAL_NUM_SIZE];
  float temperature;
  unsigned int temp1[One_Wire_Device_Number_MAX];
  ...
  while(One_Wire_Reset()) {
    DWT_Delay(500000);
  }
  ...
  DS18B20_Search_Rom2(&cnt,&rom_sn);
  ...
  for(j=0;j<cnt;j++) {
    DS18B20_Start_Conversion_by_ROM(&rom_sn[j]);
    res=DS18B20_Get_Conversion_Result_by_ROM_CRC(&rom_sn[j],&temp1[j]);
    if(res==One_Wire_Success) {
      temperature=(float)temp1[j]*0.0625;
    }
  }
```