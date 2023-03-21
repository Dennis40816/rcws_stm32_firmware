/*
 * lra_user_config.h
 *
 *  Created on: Feb 12, 2023
 *      Author: liusx
 */

#ifndef INC_LRA_LRA_USER_CONFIG_H_
#define INC_LRA_LRA_USER_CONFIG_H_

/* predefined macros */
// comment next line if you don't need to test
#define LRA_TEST
// comment next line if you don't need to debug
#define LRA_DEBUG
// comment next line to suspend system info send to Rasp
#define LRA_SYSTEM_INFO

// define usb bMaxPower by setting USBD_MAX_POWER before usbd_def.h, 250 * 2 = 500 mA
#define USBD_MAX_POWER (250)

#endif /* INC_LRA_LRA_USER_CONFIG_H_ */
