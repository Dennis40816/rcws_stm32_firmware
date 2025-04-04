/*
 * lra_user_config.h
 *
 *  Created on: Feb 12, 2023
 *      Author: liusx
 */

#ifndef INC_LRA_LRA_USER_CONFIG_H_
#define INC_LRA_LRA_USER_CONFIG_H_

/** predefined macros **/

/* System macros */

// comment next line if you don't need to test
#define LRA_TEST

// comment next line if you don't need to debug
#define LRA_DEBUG

// comment next line to suspend system info send to Rasp
#define LRA_SYSTEM_INFO

/* USB related */

// define usb bMaxPower by setting USBD_MAX_POWER before usbd_def.h, 250 * 2 =
// 500 mA
#define USBD_MAX_POWER (250)
#define LRA_USB_BUFFER_SIZE (1024)

/* ADXL355 */

#define LRA_ACC_BUFFER_SIZE (1024)

/* PWM related */

// 128 is a constant defined by DRV2605L, 170 is default vibration freq of
// VG1040003D in Hz.
#define LRA_DRV_PWM_FREQ_COEFF (128)
#define LRA_VG1040003D_RESONANT_FREQ (170)
#define LRA_DEFAULT_RESONANT_PWM_FREQ \
  (LRA_DRV_PWM_FREQ_COEFF * LRA_VG1040003D_RESONANT_FREQ)

#define LRA_DEFAULT_USER_FREQ (5.0)

// half of 1000 (1000‰)
#define LRA_DEFAULT_PWM_DUTY (500)

/* Parser related */

#define PARSER_ERR_BUF_SIZE (32)
#define PARSER_MSG_BUF_SIZE (256)

#endif /* INC_LRA_LRA_USER_CONFIG_H_ */
