/*
 * drv2605l.h
 *
 *  Created on: Feb 9, 2023
 *      Author: Dennis
 */

#ifndef INC_DEVICES_DRV2605L_H_
#define INC_DEVICES_DRV2605L_H_

/* includes */

#include "stm32f4xx_hal.h"
#include "util/util_i2c.h"

/* exported variables */

extern const uint8_t drv2605l_default_addr;
extern const uint16_t drv2605l_default_timeout_ms;

/* macros */

#define DRV2605L_MEMADDSIZE (1)  // 1 byte

/* structs */

typedef struct DRV2605L_Struct {
  uint16_t dev_addr;
  uint16_t timeout_ms;  // should always > 0
  uint16_t en_pin;
  GPIO_TypeDef* en_port;  // set to NULL if not used
  I2C_HandleTypeDef* hi2c;
} DRV2605L_t;

/* enum */

typedef enum DRV2605L_Registers {
  DRV2605L_Status = 0x00,
  DRV2605L_Mode = 0x01,
  DRV2605L_RTP_Input = 0x02,
  DRV2605L_Lib_Selection = 0x03,
  DRV2605L_Waveform_Sequencer = 0x04,
  DRV2605L_Go = 0x0C,
  DRV2605L_ODT = 0x0D,
  DRV2605L_SPT = 0x0E,
  DRV2605L_SNT = 0x0F,
  DRV2605L_BRT = 0x10,
  DRV2605L_Audio_To_Vibe_Crtl = 0x11,
  DRV2605L_Audio_To_Vibe_MinIL = 0x12,
  DRV2605L_Audio_To_Vibe_MaxIL = 0x13,
  DRV2605L_Audio_To_Vibe_MinOD = 0x14,
  DRV2605L_Audio_To_Vibe_MaxOD = 0x15,
  DRV2605L_Rated_Voltage = 0x16,
  DRV2605L_OD_Clamp = 0x17,
  DRV2605L_A_CAL_COMP = 0x18,
  DRV2605L_A_CAL_BEMF = 0x19,
  DRV2605L_Feedback_Control = 0x1A,
  DRV2605L_Control1 = 0x1B,
  DRV2605L_Control2 = 0x1C,
  DRV2605L_Control3 = 0x1D,
  DRV2605L_Control4 = 0x1E,
  DRV2605L_Control5 = 0x1F,
  DRV2605L_OL_LRA_Period = 0x20,
  DRV2605L_VBAT = 0x21,
  DRV2605L_LRA_Period = 0x22,
  DRV2605L_Total_Reg_Num
} DRV_Regs;

/* public functions */

int8_t DRV2605L_ID_Validate(const DRV2605L_t* const pDrv);
HAL_StatusTypeDef DRV2605L_SoftReset(const DRV2605L_t* const pDrv);
HAL_StatusTypeDef DRV2605L_StandbySet(const DRV2605L_t* const pDrv);
HAL_StatusTypeDef DRV2605L_StandbyUnset(const DRV2605L_t* const pDrv);
HAL_StatusTypeDef DRV2605L_GoSet(const DRV2605L_t* const pDrv);
HAL_StatusTypeDef DRV2605L_GoUnset(const DRV2605L_t* const pDrv);
HAL_StatusTypeDef DRV2605L_Run_AutoCalibration(const DRV2605L_t* const pDrv,
                                               uint8_t* pResult);
HAL_StatusTypeDef DRV2605L_Custom_Config(const DRV2605L_t* const pDrv);
HAL_StatusTypeDef DRV2605L_SetMode_PWM(const DRV2605L_t* const pDrv);

/* public IO functions */

HAL_StatusTypeDef DRV2605L_Read_All(const DRV2605L_t* const pDrv,
                                    uint8_t* pData);

HAL_StatusTypeDef DRV2605L_Write(const DRV2605L_t* const pDrv,
                                 const DRV_Regs iaddr,
                                 uint16_t len,
                                 uint8_t* const pData);

HAL_StatusTypeDef DRV2605L_Read(const DRV2605L_t* const pDrv,
                                const DRV_Regs iaddr,
                                uint16_t len,
                                uint8_t* const pData);

HAL_StatusTypeDef DRV2605L_ReadReg(const DRV2605L_t* const pDrv,
                                   const DRV_Regs iaddr,
                                   uint8_t* pData);

HAL_StatusTypeDef DRV2605L_WriteReg(const DRV2605L_t* const pDrv,
                                    const DRV_Regs iaddr,
                                    uint8_t data);

#endif /* INC_DEVICES_DRV2605L_H_ */
