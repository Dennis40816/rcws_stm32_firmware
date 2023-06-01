# A LRA vibration module based on STM32F412RxT6 and DRV2605L

## Naming Rules

1. Pointer variables starts with a lowercase 'p'
2. Type name ends with a lowercase 't' after - (dash)

## Must check before build

1. You should check usbd_cdc related files are corrected manually in Middlewares\ST\STM32_USB_Device_Library\Class\CDC.

```c
// usbd_cdc.h
#include <lra/lra_it.h>

// usbd_cdc.c
// in static uint8_t USBD_CDC_DataIn
......

// state maintainer function add here
Your_Buf_Mainter(hcdc);

return USBD_OK;
```
