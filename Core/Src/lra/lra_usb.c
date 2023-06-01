/*
 * rasp_cmd.c
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

/* includes */
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "lra/lra_usb.h"

// for CDC_Transmit_FS
#include "usbd_cdc_if.h"

/* extern variables definitions */

uint8_t lra_usb_dtr_flag;

// current usage
uint8_t lra_usb_rx_buf[LRA_USB_BUFFER_SIZE] = {0};
uint8_t lra_usb_tx_buf[LRA_USB_BUFFER_SIZE] = {0};
uint8_t lra_usb_rx_user_buf[LRA_USB_BUFFER_SIZE] = {0};

// fucture usage
uint8_t lra_usb_tx_buf1[LRA_USB_BUFFER_SIZE] = {0};
uint8_t lra_usb_tx_buf2[LRA_USB_BUFFER_SIZE] = {0};
LRA_DualBuf_t lra_usb_tx_dbuf;

/* private variables */

static LRA_USB_Mode_t lra_usb_mode = LRA_USB_NONE_MODE;
static volatile uint8_t lra_usb_rx_flag = LRA_FLAG_UNSET;

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
 * @warning This function required user to add "\r\n" at the end of format
 *
 * @details It's a non-blocking function
 * @param format
 * @param ...
 */
void LRA_USB_SysInfo(const char* format, ...) {
#ifdef LRA_SYSTEM_INFO
#define BUF_SIZE 256

  va_list args;
  uint32_t length;
  const uint8_t header_len = 3;
  static uint8_t buffer[BUF_SIZE] = {0};

  buffer[0] = USB_IN_CMD_SYS_INFO;

  va_start(args, format);
  // header_len sfor cmd_type + data_len = 3 bytes, -1 for '\0'
  // length not include '\0'
  length = vsnprintf((char*)(buffer + 3), BUF_SIZE - header_len - 1,
                     (char*)format, args);
  va_end(args);

  buffer[1] = length >> 8;
  buffer[2] = length;

  LRA_USB_Buf_Len_Pair_t msg_send = {.pbuf = buffer,
                                     .len = length + header_len};

  LRA_USB_Send_Msg(&msg_send, LRA_FLAG_UNSET);

#undef BUF_SIZE
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
  lra_usb_rx_flag = LRA_FLAG_SET;
  memcpy(pdest, psrc, len);
}

/**
 * @brief Precheck the data going to parse are valid
 *
 * @return LRA_USB_Parse_Precheck_t
 */
LRA_USB_Parse_Precheck_t LRA_USB_Parse_Precheck(LRA_USB_Msg_t* const pmsg,
                                                uint8_t* const pbuf) {
  if (LRA_USB_Get_Rx_Flag() == LRA_FLAG_UNSET)
    return PC_RX_UNSET;

  if (pbuf == NULL || pmsg == NULL)
    return PC_DATA_PTR_IS_NULL;

  // unset rx_flag
  lra_usb_rx_flag = LRA_FLAG_UNSET;

  // assign to pmsg
  const volatile uint8_t pdata_len_H = *(pbuf + 1);
  const volatile uint8_t pdata_len_L = *(pbuf + 2);

  // XXX: assume first three bytes are always correct (cmd_type and
  // pdata_len are both valid)
  // pdata won't happen if out of bound (when data_len == 0)
  *pmsg = (LRA_USB_Msg_t){.cmd_type = *pbuf,
                          .pdata_len = pdata_len_H << 1 | pdata_len_L,
                          .pdata = pbuf + 3};

  // cmd_type valid check, see enum LRA_USB_Parse_Precheck_t
  uint8_t basic_cmd_type = (pmsg->cmd_type) & BASIC_CMD_MASK;
  if (basic_cmd_type == CMD_UNKNOWN_L || basic_cmd_type >= CMD_UNKNOWN_H)
    return PC_CMD_UNKNOWN;

  // msg is not from OUT
  if (((pmsg->cmd_type) & CMD_IS_OUT) == 0)
    return PC_MSG_NOT_FROM_OUT;

  // pdata_len check min check
  if (pmsg->pdata_len == 0)
    return PC_DATA_LEN0;

  // pdata_len max check, 5 for cmd_type + data_len + EOP
  if (pmsg->pdata_len >= LRA_USB_BUFFER_SIZE - 5)
    return PC_DATA_LEN_OOB;

  // check pmsg->pdata end with EOP ("\r\n")
  if ('\r' != *(pmsg->pdata + (pmsg->pdata_len - 2)) ||
      '\n' != *(pmsg->pdata + (pmsg->pdata_len - 1)))
    return PC_EOP_NOT_FOUND;

  // pdata_len constant len check
  if (pmsg->cmd_type & CMD_DATA_LEN_CONST) {
    switch (basic_cmd_type) {
      case CMD_INIT:
      case CMD_SWITCH_MODE:
      case CMD_RESET_DEVICE:
        if (pmsg->pdata_len != lra_usb_constmsg_out_len[basic_cmd_type])
          return PC_DATA_LEN_MISMATCH;
        break;
      default:
        break;
    }
  }

  return PC_ALL_PASS;
}

/**
 * @brief Get static variable rx_flag
 *
 * @return uint8_t
 */
uint8_t LRA_USB_Get_Rx_Flag() {
  return lra_usb_rx_flag;
}

/**
 * @brief This function is for writing acceleration data to host, which need a
 *        function to avoid big size memcpy.
 *
 * @param cmd_type
 * @param pbuf buffer head address of a given buffer
 * @param data_len pure data length, not including header and EOP (total 5
 * bytes)
 * @return uint8_t
 *
 * @warning (uint8_t *)pdata should be at (pbuf + 3)
 */
HAL_StatusTypeDef LRA_USB_Generate_IN_Msg_Stack(LRA_USB_IN_Cmd_t cmd_type,
                                                uint8_t* pbuf,
                                                uint16_t data_len,
                                                LRA_USB_Buf_Len_Pair_t* in_msg,
                                                LRA_Flag_t add_eop) {
  /* user should check max_len here */
  const uint8_t header_len = 3;
  const uint8_t eop_len = add_eop ? 2 : 0;
  uint8_t package_info_len = header_len + eop_len;

  if (data_len + package_info_len > LRA_USB_BUFFER_SIZE)
    return HAL_ERROR;

  if (pbuf != NULL && data_len != 0) {
    // make msg become IN type
    *pbuf = cmd_type & NO_OUT_CMD_MASK;

    // calculate len including \r\n
    *(pbuf + 1) = (uint8_t)(data_len + eop_len) >> 8;
    *(pbuf + 2) = (uint8_t)(data_len + eop_len);

    if (add_eop) {
      *(pbuf + 3 + data_len) = '\r';
      *(pbuf + 3 + data_len + 1) = '\n';
    }

    in_msg->pbuf = pbuf;
    in_msg->len = data_len + package_info_len;
  }
  return HAL_OK;
}

/**
 * @brief
 *
 * @param cmd_type
 * @param pdata
 * @param data_len pure data length
 * @param in_msg
 * @return LRA_USB_Buf_Len_Pair_t*
 *
 * @note Here in_msg is a pointer to a struct which contains a pointer pbuf.
 * Therefore you can directly assign malloc return val to pbuf because it will
 * work like you pass a uint8_t** pointer p where *p point to pbuf.
 *
 * @warning You need to free in_msg->pbuf outside of the function.
 */
HAL_StatusTypeDef LRA_USB_Generate_IN_Msg_Heap(LRA_USB_IN_Cmd_t cmd_type,
                                               uint8_t* pdata,
                                               uint16_t data_len,
                                               LRA_USB_Buf_Len_Pair_t* in_msg,
                                               LRA_Flag_t add_eop) {
  if (pdata != NULL && data_len != 0) {
    const uint8_t header_len = 3;
    const uint8_t eop_len = add_eop ? 2 : 0;
    uint8_t package_info_len = header_len + eop_len;

    in_msg->pbuf = malloc(sizeof(uint8_t) * (data_len + package_info_len));
    if (in_msg->pbuf == NULL)
      return HAL_ERROR;

    // make msg become IN type
    *(in_msg->pbuf) = cmd_type & NO_OUT_CMD_MASK;

    *(in_msg->pbuf + 1) = (uint8_t)(data_len + eop_len) >> 8;
    *(in_msg->pbuf + 2) = (uint8_t)(data_len + eop_len);

    memcpy(in_msg->pbuf + 3, pdata, data_len);

    if (add_eop) {
      *(in_msg->pbuf + 3 + data_len) = '\r';
      *(in_msg->pbuf + 3 + data_len + 1) = '\n';
    }

    in_msg->len = data_len + package_info_len;
  }

  return HAL_OK;
}

/**
 * @brief Interface of USB transmition
 *
 * @param pair
 * @param block_flag
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef LRA_USB_Send_Msg(const LRA_USB_Buf_Len_Pair_t* const pair,
                                   const LRA_Flag_t block_flag) {
  if (pair == NULL || pair->pbuf == NULL || pair->len == 0)
    return HAL_ERROR;

  USBD_StatusTypeDef ret;
  do {
    ret = CDC_Transmit_FS(pair->pbuf, pair->len);
  } while (block_flag && (ret == USBD_BUSY));

  return (ret == USBD_OK) ? HAL_OK : HAL_ERROR;
}
