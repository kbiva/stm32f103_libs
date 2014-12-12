# README #

* lib_ENC28J60
  
ENC28J60 ethernet with TCPIP stack library. Depends on lib_delay.
Ported from http://we.easyelectronics.ru/electro-and-pc/podklyuchenie-mikrokontrollera-k-lokalnoy-seti.html
Tested with ENC28J60 board.

### How do I get set up? ###

SPI and CS pin must be configured prior calling lan_init();
Adjustable parameters:

* SPI number
* CS port and pin

### Usage example ###
```C
  lan_init(SPI1,GPIOB,GPIO_Pin_7);
  ...	
  while(1) {
    lan_poll();
    if(lan_up()) {
      BSP_LED_On(LED2);
    }
  }
```
