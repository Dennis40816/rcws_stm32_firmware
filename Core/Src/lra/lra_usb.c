/*
 * rasp_cmd.c
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

/* includes */ 
#include "string.h"
#include "lra/lra_usb.h"
// to use USB_CDC_T
#include "usbd_cdc_if.h"

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
void LRA_Debug_USB_Write(const char* pString) {
  uint8_t len = strnlen(pString, 255);
  uint8_t *pBuf = pString;

  CDC_Transmit_FS(pBuf, len);
}

#endif

/* private variables */
uint8_t lra_usb_mode = LRA_USB_CRTL_MODE;






