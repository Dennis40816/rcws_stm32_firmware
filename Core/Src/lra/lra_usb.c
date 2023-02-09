/*
 * rasp_cmd.c
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

/* includes */ 
#include <string.h>
#include <stdarg.h>

#include "lra/lra_usb.h"

// to use CDC_Transmit_FS
#include "usbd_cdc_if.h"

// extern variables
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

// public functions
void LRA_USB_Init(void) {
  lra_usb_mode = LRA_USB_CRTL_MODE;
}

void LRA_Modify_USB_Mode(uint8_t mode) {

  // TODO: safe check for mode

  lra_usb_mode = mode;
}

#ifdef LRA_DEBUG

/* public debug functions */
void LRA_USB_Print(const char *format, ...) {
  va_list args;
  uint32_t length;

  va_start(args, format);
  length = vsnprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, (char *)format, args);
  va_end(args);
  CDC_Transmit_FS(UserTxBufferFS, length);
}

#endif

/* private variables */
uint8_t lra_usb_mode = LRA_USB_CRTL_MODE;






