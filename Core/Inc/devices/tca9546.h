/*
 * tca9546.h
 *
 *  Created on: Feb 9, 2023
 *      Author: Dennis
 */

#ifndef INC_DEVICES_TCA9546_H_
#define INC_DEVICES_TCA9546_H_

/* includes */

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "util/util_i2c.h"

/* exported variables */

extern const uint8_t tca9546_addr_default;
extern uint16_t tca9546_timeout_default_ms;

/* macros */
#define TCA9546_CH_NUM (4)

// tca9546 channel mapping

/*
This lib ch     TCA9546 registers
     0     ->     no channel
     1     ->   channel 0 (SD0)
     2     ->   channel 1 (SD1)
     3     ->   channel 2 (SD2)
     4     ->   channel 3 (SD3)
*/

/* typedef */
typedef struct TCA9546PWAR {
  uint8_t ch;  // channel，應介在 0 ~ 4 之間，0 表示沒有 channel 被選擇 (default
               // state)
  uint8_t dev_addr;
  uint16_t timeout_ms;  // should > 0
  uint16_t reset_pin;   // reset pin, e.g GPIO_PIN_1
  GPIO_TypeDef* reset_port;  // reset port, e.g GPIOA
  I2C_HandleTypeDef* hi2c;
} TCA9546_t;

/* public functions */
HAL_StatusTypeDef TCA_Modify_CH(TCA9546_t* const pTca, const uint8_t ch);
int8_t TCA_Get_CH(TCA9546_t* const pTca);
HAL_StatusTypeDef TCA_Reset(TCA9546_t* const pTca);

#endif /* INC_DEVICES_TCA9546_H_ */
