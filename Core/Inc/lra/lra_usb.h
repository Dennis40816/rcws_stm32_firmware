/*
 * rasp_cmd.h
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

#ifndef INC_RASP_CMD_H_
#define INC_RASP_CMD_H_

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

  LRA_USB_NONE_MODE = -1,

  /**
   * @brief 當 usb 處於此模式，允許修改內部所有的狀態，對 Rasp
   * 的傳輸就變成一問一答，不會主動傳輸加速度等資訊給 Rasp，應該以
   * LRA_Modify_USB_Mode() 修改
   */
  LRA_USB_CRTL_MODE = 0x00,
  /**
   * @brief 當 usb 處於此模式，僅供大量傳輸內部狀態給 Rasp而不允許從 Rasp
   * 的設備修改指令，允許的 Rasp 指令僅有切換模式指令和 PWM 更新指令，應該以
   * LRA_Modify_USB_Mode() 修改
   */
  LRA_USB_DATA_MODE = 0x01,
} LRA_USB_Mode_t;

/* public functions */

HAL_StatusTypeDef LRA_USB_Init();
HAL_StatusTypeDef LRA_Modify_USB_Mode(LRA_USB_Mode_t mode);
LRA_USB_Mode_t LRA_Get_USB_Mode(void);
void LRA_USB_Print(const char* format, ...);

#endif /* INC_RASP_CMD_H_ */
