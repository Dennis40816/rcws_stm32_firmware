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

typedef struct LRA_I2C_Devices_Struct {
  TCA9546_t* pTca;
  DRV2605L_t* pDrvX;
  DRV2605L_t* pDrvY;
  DRV2605L_t* pDrvZ;
}LRA_I2C_Devices_t;

#endif /* INC_LRA_LRA_I2C_H_ */
