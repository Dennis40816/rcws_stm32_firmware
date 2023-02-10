/*
 * lra_i2c.h
 *
 *  Created on: Feb 9, 2023
 *      Author: Dennis
 */

#ifndef INC_LRA_LRA_I2C_H_
#define INC_LRA_LRA_I2C_H_

/* includes */

#include "devices/tca9546.h"
#include "devices/drv2605l.h"

/* structures */

typedef struct TCA9546_DRV2605L_I2C_Device_Pair_Struct {
  uint8_t located_ch; // should between 1 to 4 (details see tca9546.h)
  TCA9546_t* tca;
  DRV2605L_t* drv;
} TCA_DRV_Pair_t;

typedef struct LRA_I2C_Devices_Struct {
  TCA9546_t* pTca;
  DRV2605L_t* pDrvX;
  DRV2605L_t* pDrvY;
  DRV2605L_t* pDrvZ;
}LRA_I2C_Devices_t;

#endif /* INC_LRA_LRA_I2C_H_ */
