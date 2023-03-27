/*
 * rasp_cmd.h
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

#ifndef INC_RASP_CMD_H_
#define INC_RASP_CMD_H_

/* defines */

#define LRA_USB_DTR_SET (LRA_FLAG_SET)

/* includes */

#include "stm32f4xx_hal.h"

// includes user defined macros, should be the top of included files
#include "lra/lra_user_config.h"
#include "lra/lra_util.h"

/* exported variables */

extern uint8_t lra_usb_dtr_flag;

extern uint8_t lra_usb_rx_buf[];
extern uint8_t lra_usb_tx_buf[];

extern uint8_t lra_usb_rx_user_buf[];

// future usage
extern LRA_DualBuf_t lra_usb_tx_dbuf;

/* macros */

/* enums */

typedef enum {

  LRA_USB_NONE_MODE = 0x00,

  LRA_USB_WAIT_FOR_INIT_MODE = 0x01,

  /**
   * @brief 當 usb 處於此模式，允許修改內部所有的狀態，對 Rasp
   * 的傳輸就變成一問一答，不會主動傳輸加速度等資訊給 Rasp，應該以
   * LRA_Modify_USB_Mode() 修改
   */
  LRA_USB_CRTL_MODE = 0x02,
  /**
   * @brief 當 usb 處於此模式，僅供大量傳輸內部狀態給 Rasp而不允許從 Rasp
   * 的設備修改指令，允許的 Rasp 指令僅有切換模式指令和 PWM 更新指令，應該以
   * LRA_Modify_USB_Mode() 修改
   */
  LRA_USB_DATA_MODE = 0x03,
} LRA_USB_Mode_t;

typedef enum {
  LRA_USB_PARSE_PRECHECK_OK = 0x0,
  LRA_USB_PARSE_PRECHECK_UNKNOWN = 0x01,
  LRA_USB_PARSE_PRECHECK_RX_UNSET = 0x02,
  LRA_USB_PARSE_PRECHECK_NULLERR = 0x03,
  LRA_USB_PARSE_PRECHECK_LEN0 = 0x04,  // LRA_USB_Msg_t->pdata_len == 0
  LRA_USB_PARSE_PRECHECK_EOFERR = 0x05,
  LRA_USB_PARSE_PRECHECK_LEN_MISSMATCH = 0x06,
} LRA_USB_Parse_Precheck_t;

typedef enum {
  LRA_USB_PARSE_FAIL_IN_PRECHECK = -1,
  LRA_USB_PARSE_FAIL_IN_DATA = 0x00,
  LRA_USB_PARSE_INIT_OK = 0x01,
  LRA_USB_PARSE_UPDATE_PWM_OK = 0x02,
  LRA_USB_PARSE_UPDATE_REG_OK = 0x03,
} LRA_USB_Parse_Result_t;

typedef enum { LRA_FLAG_UNSET, LRA_FLAG_SET } LRA_Flag_t;

/**
 * @brief it's a uint8_t number
 *
 * @warning update LRA_USB_Parse_Precheck() and LRA_USB_Main_Parser() once you
 * add a new cmd
 *
 */
typedef enum {
  LRA_USB_CMD_INIT = 0x00,
  LRA_USB_CMD_UPDATE_PWM = 0x01,
  LRA_USB_CMD_UPDATE_REG =
      0x02,  // TODO: 更新哪一個的 register 放在 data 裡面好了
  LRA_USB_CMD_RESET_REG = 0x03,
  LRA_USB_CMD_RESET_STM32 = 0x04,
} LRA_USB_Cmd_t;

// DL for data len
// FIXME: Need to fix this
typedef enum {
  LRA_USB_OUT_INIT_DL = sizeof(LRA_USB_OUT_INIT_STR) - 1,
  LRA_USB_OUT_UPDATE_PWM_DL = 0x01,
  LRA_USB_OUT_UPDATE_REG_DL = 0x01,
  LRA_USB_OUT_RESET_REG_DL = 0x01,
  LRA_USB_OUT_RESET_STM32_DL = sizeof(LRA_USB_OUT_RESET_STM32_STR) - 1,
} LRA_USB_Out_DL_t;

typedef enum {
  LRA_USB_IN_INIT_DL = sizeof(LRA_USB_IN_INIT_STR) - 1,
  LRA_USB_IN_UPDATE_PWM_DL = 0x01,
  LRA_USB_IN_UPDATE_REG_DL = 0x01,
  LRA_USB_IN_RESET_REG_DL = 0x01,
  LRA_USB_IN_RESET_STM32_DL =
      sizeof(LRA_USB_IN_RESET_STM32_STR) - 1,  // no return
} LRA_USB_In_DL_t;

/* structs */

/**
 * @brief The msg contrains 1 byte cmd_type, 2 bytes length information
 *        and pdata pointer.
 * @param cmd_type Defined in enum LRA_USB_Cmd_t
 * @param pdata_len The length of pdata
 * @param pdata Where to start parsing data
 */
typedef struct {
  volatile uint8_t cmd_type;
  volatile uint16_t pdata_len;  // (H << 1) | L
  volatile uint8_t* pdata;
} LRA_USB_Msg_t;

/* public functions */

HAL_StatusTypeDef LRA_USB_Init();
HAL_StatusTypeDef LRA_Modify_USB_Mode(LRA_USB_Mode_t mode);
LRA_USB_Mode_t LRA_Get_USB_Mode(void);
void LRA_USB_Print(const char* format, ...);
void LRA_USB_Buffer_Copy(uint8_t* pdes, uint8_t* psrc, uint16_t len);
LRA_USB_Parse_Precheck_t LRA_USB_Parse_Precheck(LRA_USB_Msg_t* const pmsg,
                                                uint8_t* const pbuf);
uint8_t LRA_USB_Get_Rx_Flag();

#endif /* INC_RASP_CMD_H_ */
