/*
 * rasp_cmd.h
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

#ifndef INC_RASP_CMD_H_
#define INC_RASP_CMD_H_

// includes
#include "lra/lra_main.h"

// extern variables
extern uint8_t lra_usb_mode;

// LRA_main board modes macros
/**
 * @brief 當 usb 使用此模式，允許修改內部所有的狀態，對 Rasp 的資料傳輸即刻停止，應該以 LRA_Modify_USB_Mode() 修改
 * 
 */
#define LRA_USB_CRTL_MODE 0x00
/**
 * @brief 當 usb 使用此模式，僅供傳輸內部狀態給 Rasp，允許的指令僅有切換模式指令，應該以 LRA_Modify_USB_Mode() 修改
 * 
 */
#define LRA_USB_DATA_MODE 0x01


// LRA usb public functions
/**
 * @brief 由 main.c 呼叫，初始化 lra usb 相關設定用
 * 
 */
void LRA_USB_Init(void);
/**
 * @brief 更改 lra usb mode 及修改依賴項
 * 
 * @param mode lra usb 的 mode，定義在 lra_usb.h 中
 */
void LRA_Modify_USB_Mode(uint8_t mode);

// LRA usb debug functions
#ifdef LRA_DEBUG

void LRA_Debug_USB_Write(const char* pString);

#endif

#endif /* INC_RASP_CMD_H_ */
