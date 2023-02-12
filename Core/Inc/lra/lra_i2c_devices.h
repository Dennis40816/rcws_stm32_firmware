/*
 * lra_i2c.h
 *
 *  Created on: Feb 9, 2023
 *      Author: Dennis
 */

#ifndef INC_LRA_LRA_I2C_H_
#define INC_LRA_LRA_I2C_H_

/* includes */

// includes user defined macros, should be the top of included files
#include "lra/lra_user_config.h"

#include "devices/drv2605l.h"
#include "devices/tca9546.h"

/* macros */

#define LRA_MOTOR_NUM 3

/* structures */

typedef struct TCA9546_DRV2605L_I2C_Device_Pair_Struct {
  uint8_t located_ch;  // should between 1 to 4 (details see tca9546.h)
  TCA9546_t* pTca;
  DRV2605L_t* pDrv;
} TCA_DRV_Pair_t;

/**
 * Init:
 * LRA_I2C_Devs_t i2cdev = {
 *  .pDevPair = {
 *   [0] = &pair1,
 *   [1] = &pair2,
 *   ...
 *  }
 * }
 */
typedef struct LRA_I2C_Devices_Struct {
  const uint8_t pair_count;                 // number of TCA_DRV_Pair_t
  TCA_DRV_Pair_t* pDevPair[LRA_MOTOR_NUM];  // array of TCA_DRV_Pair_t, should
                                            // be continuous array

} LRA_I2C_Devs_t;

/* public functions */

HAL_StatusTypeDef LRA_I2C_Devs_Init(const LRA_I2C_Devs_t* const pDevs);
HAL_StatusTypeDef TCA_DRV_Pair_SwitchCH(const TCA_DRV_Pair_t* const pDevPair);

/* IO functions */

HAL_StatusTypeDef TCA_DRV_Pair_Write(const TCA_DRV_Pair_t* const pDevPair,
                                     const DRV_Regs iaddr,
                                     const uint16_t len,
                                     uint8_t* const pData);

HAL_StatusTypeDef TCA_DRV_Pair_Read(const TCA_DRV_Pair_t* const pDevPair,
                                    const DRV_Regs iaddr,
                                    const uint16_t len,
                                    uint8_t* const pData);

HAL_StatusTypeDef TCA_DRV_Pair_WriteReg(const TCA_DRV_Pair_t* const pDevPair,
                                        const DRV_Regs iaddr,
                                        const uint8_t data);

HAL_StatusTypeDef TCA_DRV_Pair_ReadReg(const TCA_DRV_Pair_t* const pDevPair,
                                       const DRV_Regs iaddr,
                                       uint8_t* const data);

#endif /* INC_LRA_LRA_I2C_H_ */
