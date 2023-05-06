/*
 * lra_msg.c
 *
 *  Created on: Mar 26, 2023
 *      Author: Dennis Liu
 */

/* includes */

#include "lra/lra_user_config.h"

#include "lra/lra_usb.h"
#include "stm32f4xx_hal.h"
#include "string.h"

/* defines */

#define SINGLE_CHAR_MSG_LEN (3)

/** static variables definitions **/

/* USB OUT msg definitions */

static const char msg_init[] = "MFIL-RCWS-USB init\r\n";
static const char msg_newline[] = "\r\n";

/* extern variables definitions */

const uint8_t* lra_usb_constmsg_out[32] = {[CMD_INIT] = (uint8_t*)msg_init};
const uint8_t* lra_usb_constmsg_in[32] = {
    [CMD_INIT] = (uint8_t*)msg_init,
    [CMD_RESET_DEVICE] = (uint8_t*)msg_newline,
    [CMD_SWITCH_MODE] = (uint8_t*)msg_newline,
};
const uint16_t lra_usb_constmsg_out_len[32] = {
    [CMD_INIT] = sizeof(msg_init) - 1,
    [CMD_SWITCH_MODE] = SINGLE_CHAR_MSG_LEN,
    [CMD_RESET_DEVICE] = SINGLE_CHAR_MSG_LEN};
const uint16_t lra_usb_constmsg_in_len[32] = {
    [CMD_INIT] = sizeof(msg_init) - 1,
    [CMD_SWITCH_MODE] = SINGLE_CHAR_MSG_LEN,
    [CMD_RESET_DEVICE] = SINGLE_CHAR_MSG_LEN};