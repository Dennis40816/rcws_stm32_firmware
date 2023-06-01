/**
 * @file lra_it.h
 * @author DennisLiu16
 * @date 2023-05-25
 *
 * @copyright Copyright (c) 2023
 *
 */

/* config includes */
#include <lra/lra_user_config.h>

/* includes */
#include <devices/adxl355.h>
#include <lra/lra_usb.h>
#include <lra/lra_util.h>

/* extern variables */

extern uint8_t acc_should_send_flag;

/* public function pointer */

extern HAL_StatusTypeDef (*const LRA_IT_ADXL355_DRDY_Callback_P)(
    ADXL355_t* const,
    LRA_DualBuf_t* const);

/* public functions */

void LRA_IT_LED_Flash_Every(const uint32_t it_times);
