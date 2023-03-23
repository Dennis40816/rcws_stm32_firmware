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
static volatile uint8_t lra_usb_rx_flag = LRA_USB_RX_UNSET;

/* private functions declarations */

/* public functions */

/**
 * @brief 由 lra_main.c 呼叫，將 USB 模式設定成 LRA_USB_CTRL_MODE
 *
 */
HAL_StatusTypeDef LRA_USB_Init() {
  return LRA_Modify_USB_Mode(LRA_USB_NONE_MODE);
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

/**
 * @brief Copy data from psrc to pdest with desired len
 *
 * @param pdes
 * @param psrc
 * @param len
 *
 * @warning This function will change lra_usb_rx_flag to set, should only use in
 * CDC_Receive_FS()
 */
void LRA_USB_Buffer_Copy(uint8_t* pdest, uint8_t* psrc, uint16_t len) {
  lra_usb_rx_flag = LRA_USB_RX_SET;
  memcpy(pdest, psrc, len);
}

/**
 * @brief Precheck the data going to parse are valid
 *
 * @return LRA_USB_Parse_Check_t
 */
LRA_USB_Parse_Check_t LRA_USB_Parse_Precheck(LRA_USB_Msg_t* const pmsg) {
  // unset rx_flag
  lra_usb_rx_flag = LRA_USB_RX_UNSET;

  // pdata_len is 0
  if (pmsg->pdata_len == 0)
    return LRA_USB_PARSE_LEN0;

  // check pmsg->pdata end of "\r\n"
  if ('\r' != *(pmsg->pdata + (pmsg->pdata_len - 2)) ||
      '\n' != *(pmsg->pdata + (pmsg->pdata_len - 1)))
    return LRA_USB_PARSE_EOFERR;

  // TODO: pdata_len check
  switch (pmsg->cmd_type) {
    case LRA_USB_CMD_INIT:
      if (pmsg->pdata_len != LRA_USB_OUT_INIT_DL)
        return LRA_USB_PARSE_LEN_MISSMATCH;
      break;
    case LRA_USB_CMD_UPDATE_PWM:
      if (pmsg->pdata_len != LRA_USB_OUT_UPDATE_PWM_DL)
        return LRA_USB_PARSE_LEN_MISSMATCH;
      break;

    default:
      break;
  }

  return LRA_USB_PARSE_OK;
}

/**
 * @brief Get static variable rx_flag
 *
 * @return uint8_t
 */
uint8_t LRA_USB_Get_Rx_Flag() {
  return lra_usb_rx_flag;
}
