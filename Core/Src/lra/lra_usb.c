/*
 * rasp_cmd.c
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

/* includes */

#include <stdarg.h>
#include <string.h>

#include "lra/lra_usb.h"

// for CDC_Transmit_FS
#include "usbd_cdc_if.h"

/* extern variables definitions */

uint8_t lra_usb_dtr_flag;
uint8_t lra_usb_rx_buf[LRA_USB_BUFFER_SIZE] = {0};
uint8_t lra_usb_tx_buf[LRA_USB_BUFFER_SIZE] = {0};
uint8_t lra_usb_rx_user_buf[LRA_USB_BUFFER_SIZE] = {0};

// fucture usage
LRA_DualBuf_t lra_usb_tx_dbuf;

/* private variables */

static LRA_USB_Mode_t lra_usb_mode = LRA_USB_NONE_MODE;
static uint8_t lra_usb_rx_flag = LRA_USB_RX_UNSET;


/* public functions */

/**
 * @brief 由 lra_main.c 呼叫，將 USB 模式設定成 LRA_USB_CTRL_MODE
 *
 */
HAL_StatusTypeDef LRA_USB_Init() {
  return LRA_Modify_USB_Mode(LRA_USB_CRTL_MODE);
}

/**
 * @brief 更改 lra usb mode 及修改依賴項
 *
 * @param mode lra usb 的 mode，定義在 lra_usb.h 中
 */
HAL_StatusTypeDef LRA_Modify_USB_Mode(LRA_USB_Mode_t mode) {
  if (lra_usb_mode == mode)
    return HAL_OK;

// TODO: safe check for mode
  switch (lra_usb_mode) {
    case LRA_USB_NONE_MODE:
      break;

    default:
      break;
  }

  lra_usb_mode = mode;
  return HAL_OK;
}

/**
 * @brief Return current mode
 *
 * @return LRA_USB_Mode_t
 */
LRA_USB_Mode_t LRA_Get_USB_Mode() {
  return lra_usb_mode;
}

/**
 * @brief LRA printf-like usb print function, max length is 256 characters. Note
 * that this function maintains a static stack buffer with size 256.
 *
 * @details It's a non-blocking
 * @param format
 * @param ...
 */
void LRA_USB_Print(const char* format, ...) {
#ifdef LRA_SYSTEM_INFO
  va_list args;
  uint32_t length;

  va_start(args, format);

  static uint8_t buffer[256] = {0};

  length = vsnprintf((char*)buffer, 256, (char*)format, args);
  va_end(args);
  CDC_Transmit_FS(buffer, length);
#endif
}
