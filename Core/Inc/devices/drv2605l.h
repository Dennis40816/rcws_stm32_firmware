/*
 * drv2605l.h
 *
 *  Created on: Feb 9, 2023
 *      Author: Dennis
 */

#ifndef INC_DEVICES_DRV2605L_H_
#define INC_DEVICES_DRV2605L_H_

/* includes */

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "util/util_i2c.h"

/* exported variables */

extern const uint8_t drv2605l_addr_default;

/* macros */

#define DRV2605L_MEMADDSIZE (1) // 1 byte

/* structs */

typedef struct DRV2605L_Struct {
  uint16_t dev_addr;
  uint16_t timeout_ms;  // should > 0
  I2C_HandleTypeDef* hi2c;
}DRV2605L_t;

/* enum */

typedef enum DRV2605L_Registers {
  Status = 0x00,
  Mode = 0x01,
  RTP_Input = 0x02,
  Lib_Selection = 0x03,
  Waveform_Sequencer = 0x04,
  Go = 0x0C,
  ODT = 0x0D,
  SPT = 0x0E,
  SNT = 0x0F,
  BRT = 0x10,
  Audio_To_Vibe_Crtl = 0x11,
  Audio_To_Vibe_MinIL = 0x12,
  Audio_To_Vibe_MaxIL = 0x13,
  Audio_To_Vibe_MinOD = 0x14,
  Audio_To_Vibe_MaxOD = 0x15,
  Rated_Voltage = 0x16,
  OD_Clamp = 0x17,
  A_CAL_COMP = 0x18,
  A_CAL_BEMF = 0x19,
  Feedback_Control = 0x1A,
  Control1 = 0x1B,
  Control2 = 0x1C, 
  Control3 = 0x1D,
  Control4 = 0x1E,
  Control5 = 0x1F,
  OL_LRA_Period = 0x20,
  VBAT = 0x21,
  LRA_Period = 0x22,
  DRV_Total_Reg_Num
}DRV_Regs;

/* public functions */

HAL_StatusTypeDef DRV_SoftReset(const DRV2605L_t* const pDrv);
HAL_StatusTypeDef DRV_Go(const DRV2605L_t* const pDrv);
HAL_StatusTypeDef DRV_Stop(const DRV2605L_t* const pDrv);
HAL_StatusTypeDef DRV_Read_All(const DRV2605L_t* const pDrv, uint8_t* pData);

HAL_StatusTypeDef DRV_Write(const DRV2605L_t* const pDrv,
                            const DRV_Regs iaddr,
                            uint16_t len,
                            uint8_t* const pData);

HAL_StatusTypeDef DRV_Read(const DRV2605L_t* const pDrv,
                           const DRV_Regs iaddr,
                           uint16_t len,
                           uint8_t* const pData);

#endif /* INC_DEVICES_DRV2605L_H_ */
