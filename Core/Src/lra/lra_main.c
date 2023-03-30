/*
 * lra_main.c
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

/* includes */

// contrains user config header, should be the top of include files
#include "lra/lra_main.h"

// get some pins' macros
#include "main.h"

// includes device related headers
#include "lra/lra_i2c_devices.h"
#include "lra/lra_pwm.h"
#include "lra/lra_spi_devices.h"
#include "lra/lra_sys_error.h"
#include "lra/lra_usb.h"

// includes parser string operation related headers
#include "string.h"
#include "usbd_cdc_if.h"

/* extern */

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;

/* private function declarations */

static uint16_t LRA_USB_Main_Parser();

/* global vars */

// create devices' structs
TCA9546_t tca = {
    .ch = 0,
    .dev_addr = 0,
    .timeout_ms = 1,
    .reset_pin = TCA_NRST_Pin,
    .reset_port = TCA_NRST_GPIO_Port,
    .hi2c = &hi2c1,
};
DRV2605L_t drv_x = {
    .dev_addr = 0,
    .timeout_ms = 1,
    .en_pin = 0,
    .en_port = NULL,
    .hi2c = &hi2c1,
};
DRV2605L_t drv_y = {
    .dev_addr = 0,
    .timeout_ms = 1,
    .en_pin = 0,
    .en_port = NULL,
    .hi2c = &hi2c1,
};
DRV2605L_t drv_z = {
    .dev_addr = 0,
    .timeout_ms = 1,
    .en_pin = 0,
    .en_port = NULL,
    .hi2c = &hi2c1,
};

// create TCA9546A and DRV2605L pair structs
TCA_DRV_Pair_t tca_drv_x = {.located_ch = 1, .pDrv = &drv_x, .pTca = &tca};
TCA_DRV_Pair_t tca_drv_y = {.located_ch = 2, .pDrv = &drv_y, .pTca = &tca};
TCA_DRV_Pair_t tca_drv_z = {.located_ch = 3, .pDrv = &drv_z, .pTca = &tca};

// create I2C_Devs struct
LRA_I2C_Devs_t i2c_devs = {.pair_count = LRA_MOTOR_NUM,
                           .pDevPair = {&tca_drv_x, &tca_drv_y, &tca_drv_z}};

// create PWM array
LRA_PWM_t pwm_x = {
    .htim = &htim2,
    .ch = TIM_CHANNEL_1,
};

LRA_PWM_t pwm_y = {
    .htim = &htim2,
    .ch = TIM_CHANNEL_2,
};

LRA_PWM_t pwm_z = {
    .htim = &htim2,
    .ch = TIM_CHANNEL_3,
};

LRA_PWM_t* pwm_arr[] = {&pwm_x, &pwm_y, &pwm_z};

/* create MPU6500 instance */

/* create ADXL355 instance */
ADXL355_t adxl355 = {
    .hspi = &hspi3,
    .nss_pin = 0,
    .nss_port = NULL,
    .range = ADXL355_ACC_2G,
    .timeout_ms = 1,

    // use temp default parser, change these parameters after adjustment
    .temp_intercept_lsb = 0,
    .temp_slope = 0.0,
    .temp_intercept_Celsius = 0.0,
};

/* public functions */

void LRA_Main_EnterPoint(void) {
  // LED flash *2: enter LRA Main EnterPoint
  LRA_LED_Flash_N(2, 500);

  LRA_Main_System_Init();

  // create error status variable
  uint8_t error = 0;

  HAL_StatusTypeDef ret;

  // dtr flag assignment init
  lra_usb_dtr_flag = 0;

  tca.dev_addr = tca9546_default_addr;
  tca.timeout_ms = tca9546_default_timeout_ms;

  drv_x.dev_addr = drv2605l_default_addr;
  drv_x.timeout_ms = drv2605l_default_timeout_ms;

  drv_y.dev_addr = drv2605l_default_addr;
  drv_y.timeout_ms = drv2605l_default_timeout_ms;

  drv_z.dev_addr = drv2605l_default_addr;
  drv_z.timeout_ms = drv2605l_default_timeout_ms;

  /* Buffer Init */
  // TODO: 移動至 usb 和 acc 中
  uint8_t lra_usb_tx_buf1[LRA_USB_BUFFER_SIZE] = {0};
  uint8_t lra_usb_tx_buf2[LRA_USB_BUFFER_SIZE] = {0};

  uint8_t lra_acc_buf1[LRA_ACC_BUFFER_SIZE] = {0};
  uint8_t lra_acc_buf2[LRA_ACC_BUFFER_SIZE] = {0};

  lra_usb_tx_dbuf = (LRA_DualBuf_t){
      .buf_full = {0, 0},
      .buf_index = {0, 0},
      .buf_size = {LRA_USB_BUFFER_SIZE, LRA_USB_BUFFER_SIZE},
      .cur_buf = 0,
      .dbuf = {lra_usb_tx_buf1, lra_usb_tx_buf2},
  };

  lra_acc_dbuf = (LRA_DualBuf_t){
      .buf_full = {0, 0},
      .buf_index = {0, 0},
      .buf_size = {LRA_ACC_BUFFER_SIZE, LRA_ACC_BUFFER_SIZE},
      .cur_buf = 0,
      .dbuf = {lra_acc_buf1, lra_acc_buf2},
  };

  /* USB Init */
  ret = LRA_USB_Init(LRA_USB_CRTL_MODE);
  if (ret != HAL_OK)
    error |= (1 << LRA_INIT_ERR_USB);

  while (LRA_Get_USB_Mode() != LRA_USB_CRTL_MODE) {
    /**
     * @brief Wait for host send LRA_USB_CMD_Init msg, this will make USB state
     * becomes LRA_USB_WAIT_FOR_INIT_MODE (changes made in usbd_cdc_if.c when
     * receive DTR signal). Keep going if receive correct init cmd.
     *
     */
    if (LRA_USB_Main_Parser() == USB_OUT_CMD_INIT)
      break;
  }

  /* I2C devs init */
  ret = LRA_I2C_Devs_Init(&i2c_devs);
  if (ret != HAL_OK)
    error |= (1 << LRA_INIT_ERR_I2C_DEVS);

  /* PWM init */
  for (int i = 0; i < LRA_MOTOR_NUM; i++) {
    ret = Lra_PWM_Init(pwm_arr[i], LRA_DEFAULT_PWM_FREQ, LRA_DEFAULT_PWM_DUTY);
    if (ret != HAL_OK)
      error |= (1 << LRA_INIT_ERR_PWM);
  }

  /* TODO: MPU6500 init */

  if (ret != HAL_OK)
    error |= (1 << LRA_INIT_ERR_MPU6500);

  /* TODO: ADXL355 init */
  ret = ADXL355_Init(&adxl355);
  if (ret != HAL_OK)
    error |= (1 << LRA_INIT_ERR_ADXL355);

  // you should check error code here
  if (error)
    LRA_USB_SysInfo("Init error code: %x", error);
  else
    LRA_USB_SysInfo("Init success\r\n");

  /* parameters used in main loop */

  /**
   * @brief pwm related variables \r\n
   *
   * pwm_flag
   *
   * val 0 0 0 0 0 0 0 0
   *     - - - - - - - -
   * bit 7 6 5 4 3 2 1 0
   *
   * 0th bit: update pwm freq
   * 1th bit: update pwm duty cycle
   *
   * check enum LRA_PWM_FLAG in lra_main.h also
   */
  uint8_t pwm_flag = 0;

  // CMD
  uint32_t pwm_freq_cmd[LRA_MOTOR_NUM] = {
      LRA_DEFAULT_PWM_FREQ, LRA_DEFAULT_PWM_FREQ, LRA_DEFAULT_PWM_FREQ};
  uint16_t pwm_duty_cmd[LRA_MOTOR_NUM] = {
      LRA_DEFAULT_PWM_DUTY, LRA_DEFAULT_PWM_DUTY, LRA_DEFAULT_PWM_DUTY};

  // System enable
  for (int i = 0; i < LRA_MOTOR_NUM; i++) {
    Lra_PWM_Enable(pwm_arr[i]);
  }

  // Flash *2: init end
  LRA_LED_Flash_N(2, 500);

// Test code
#ifdef LRA_TEST
  LRA_USB_SysInfo("Test started\r\n");

  LRA_USB_SysInfo("Read \r\n");

#endif

  while (1) {
    /* Loop update */
    LRA_USB_Main_Parser();
    /* usb parser */

    // usb receive flag & parser

    // usb tranmit flag

    // cmd update flag

    // internal update flag

    // --------------------------------

    /* feedback */

    /* calculate new pwm signal */

    /* update pwm singal */

    /* transport data to Rasp */
  }
}

/**
 * @brief Init STM32 related handles
 *
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef LRA_Main_System_Init(void) {
  // System interrupt related
  HAL_TIM_Base_Start_IT(&htim6);
  return HAL_OK;
}

/* private functions */

/**
 * @brief
 *
 * @return uint16_t
 */
static uint16_t LRA_USB_Main_Parser() {
  // TODO: get correct buffer if using dual buffers
  uint8_t* const pbuf = lra_usb_rx_user_buf;

  LRA_USB_Msg_t pmsg;

  f_ptr func_callback = NULL;

  uint8_t* tx_data_buf = NULL;
  uint16_t tx_data_buf_len = 0;

  /* state flags, default is everything fine
   * --------------------------------
   * parse format error: data 格式不符合設定格式
   * parse content error: 解讀後的 data 數值超過預期範圍，會數值為未定義
   * mode error: It's not allow to parse and execute the given cmd_type in
   * currect mode
   *
   * 上述三種錯誤都會傳遞
   *
   */
  uint8_t internal_operation_error_flag = LRA_FLAG_UNSET;
  uint8_t parse_content_error_flag = LRA_FLAG_UNSET;
  uint8_t mode_error_flag = LRA_FLAG_UNSET;

  // add EOP (\r\n) at the end of data buffer
  uint8_t return_msg_add_eop_flag = LRA_FLAG_UNSET;

  // LRA_USB_Parse_Precheck() will unset lra_usb_rx_flag no matter the precheck
  // pass or not, so call it before you parse the data
  LRA_USB_Parse_Precheck_t precheck_status =
      LRA_USB_Parse_Precheck(&pmsg, pbuf);

  // Precheck rx unset
  if (precheck_status == PC_RX_UNSET)
    return PR_RX_UNSET;

  // precheck fail, send back error msg
  if (precheck_status != PC_ALL_PASS) {
    uint8_t cmd_is_unknown = (precheck_status == PC_CMD_UNKNOWN);

    // format msg, USB_OUT_CMD_PARSE_ERR, data length 2
    uint8_t precheck_msg[32] = {0};
    const uint8_t data_len = 2;

    LRA_USB_Buf_Len_Pair_t msg_send;

    precheck_msg[3] = PR_PRECHECK_FAIL >> 8;
    precheck_msg[4] = (cmd_is_unknown) ? 0 : pmsg.cmd_type;

    // use stack, not free required
    LRA_USB_Generate_IN_Msg_Stack(USB_IN_CMD_PARSE_ERR, precheck_msg, data_len,
                                  &msg_send, LRA_FLAG_SET);

    /* send msg */
    LRA_USB_Send_Msg(&msg_send, LRA_FLAG_SET);

    return (cmd_is_unknown) ? PR_PRECHECK_FAIL
                            : (PR_PRECHECK_FAIL | pmsg.cmd_type);
  }

  /* start to parse pdata */
  volatile uint8_t* const cursor = pmsg.pdata;

  /**
   * parser preprocess
   *
   * 1. 有固定字串可以比對的，進行比對 (應該只有 init)
   * 2. 需要執行內部程式，執行
   *
   */
  switch (pmsg.cmd_type & BASIC_CMD_MASK) {
    case CMD_SYS_INFO:
    case CMD_PARSE_ERR:
      // usually we don't get here, ignore
      break;

    case CMD_SWITCH_MODE: {
      uint8_t mode = *cursor;

      switch (mode) {
        case LRA_USB_CRTL_MODE:
        case LRA_USB_DATA_MODE:
          LRA_Modify_USB_Mode(mode);
          tx_data_buf = lra_usb_constmsg_in[CMD_SWITCH_MODE];
          tx_data_buf_len = lra_usb_constmsg_in_len[CMD_SWITCH_MODE];
          break;
        default:
          parse_content_error_flag = LRA_FLAG_SET;
          break;
      }

      break;
    }
    case CMD_INIT:
      // mode check
      if (LRA_Get_USB_Mode() != LRA_USB_WAIT_FOR_INIT_MODE) {
        mode_error_flag = LRA_FLAG_SET;
        break;
      }

      // content compare
      if (memcmp(pmsg.pdata, lra_usb_constmsg_out[CMD_INIT], pmsg.pdata_len) ==
          0) {
        // change STM32 state
        LRA_Modify_USB_Mode(LRA_USB_CRTL_MODE);

        // set tx, len
        tx_data_buf = lra_usb_constmsg_in[CMD_INIT];
        tx_data_buf_len = lra_usb_constmsg_in_len[CMD_INIT];
      } else {
        parse_content_error_flag = LRA_FLAG_SET;
      }
      break;

    /**
     * Update device's registers.
     *
     * first byte: device
     * second byte: internal address begin
     * third byte: internal address end
     * fourth byte: data start here
     *
     */
    case CMD_UPDATE_REG:
      if (LRA_Get_USB_Mode() != LRA_USB_CRTL_MODE) {
        mode_error_flag = LRA_FLAG_SET;
        break;
      }

      // TODO

      break;

    case CMD_GET_REG:
      if (LRA_Get_USB_Mode() != LRA_USB_CRTL_MODE) {
        mode_error_flag = LRA_FLAG_SET;
        break;
      }

      uint8_t device_index = *cursor;
      if (!LRA_Device_Is_Valid(device_index)) {
        parse_content_error_flag = LRA_FLAG_SET;
        break;
      }

      switch (device_index) {
        case LRA_DEVICE_MPU6500:
          break;

        case LRA_DEVICE_ADXL355:
          // TODO: range check func

          // read to tmp and assign to tx_buf

          return_msg_add_eop_flag = LRA_FLAG_SET;
          break;

        case LRA_DEVICE_DRV2605L_X:
        case LRA_DEVICE_DRV2605L_Y:
        case LRA_DEVICE_DRV2605L_Z:
          /* TODO */
          break;

        default:
          parse_content_error_flag = LRA_FLAG_SET;
          break;
      }

      break;

    case CMD_RESET_DEVICE: {
      if (LRA_Get_USB_Mode() != LRA_USB_CRTL_MODE) {
        mode_error_flag = LRA_FLAG_SET;
        break;
      }

      uint8_t device_index = *cursor;
      if (!LRA_Device_Is_Valid(device_index)) {
        parse_content_error_flag = LRA_FLAG_SET;
        break;
      }

      switch (device_index) {
        case LRA_DEVICE_STM32:
          func_callback = NVIC_SystemReset;
          break;

        case LRA_DEVICE_MPU6500:
          // TODO
          break;

        case LRA_DEVICE_ADXL355: {
          uint8_t reset_status;
          reset_status = ADXL355_Reset_Device(&adxl355);
          if (reset_status != HAL_OK)
            internal_operation_error_flag = LRA_FLAG_SET;
          break;
        }

        case LRA_DEVICE_DRV2605L_X:
        case LRA_DEVICE_DRV2605L_Y:
        case LRA_DEVICE_DRV2605L_Z: {
          uint8_t drv_dev_index = device_index - LRA_DEVICE_DRV2605L_X;
          const DRV2605L_t* const pdrv = i2c_devs.pDevPair[drv_dev_index]->pDrv;

          uint8_t reset_status;
          reset_status = DRV2605L_SoftReset(pdrv);
          if (reset_status != HAL_OK)
            internal_operation_error_flag = LRA_FLAG_SET;
          break;
        }

        default:
          parse_content_error_flag = LRA_FLAG_SET;
          break;
      }
      tx_data_buf = lra_usb_constmsg_in[CMD_RESET_DEVICE];
      tx_data_buf_len = lra_usb_constmsg_in_len[CMD_RESET_DEVICE];

      break;
    }

    case CMD_RUN_AUTOCALIBRATE:
      if (LRA_Get_USB_Mode() != LRA_USB_CRTL_MODE) {
        mode_error_flag = LRA_FLAG_SET;
        break;
      }
      /* TODO: call auto calibration */
      break;

    case CMD_UPDATE_ARG:
      if (LRA_Get_USB_Mode() != LRA_USB_DATA_MODE) {
        mode_error_flag = LRA_FLAG_SET;
        break;
      }
      /* TODO: update frequency and arg */
      break;

    case CMD_UPDATE_ACC:  // XXX
      if (LRA_Get_USB_Mode() != LRA_USB_DATA_MODE) {
        mode_error_flag = LRA_FLAG_SET;
        break;
      }
      /* something goes wrong, you will never get here */

      break;
  }

  /* if any flag trigger, send USB_IN_CMD_PARSE_ERR msg */
  if (parse_content_error_flag || mode_error_flag ||
      internal_operation_error_flag) {
    uint16_t parser_status = pmsg.cmd_type;

    if (mode_error_flag) {
      parser_status |= PR_CURRENT_MODE_FORBIDDEN_FAIL;
    } else if (parse_content_error_flag) {
      parser_status |= PR_DATA_PARSE_CONTENT_FAIL;
    } else if (PR_INTERNAL_OPERATION_FAIL) {
      parser_status |= PR_INTERNAL_OPERATION_FAIL;
    }

    uint8_t parse_err_msg[32];
    const uint8_t parse_err_msg_len = 2;
    parse_err_msg[3] = parser_status >> 8;
    parse_err_msg[4] = parser_status;

    LRA_USB_Buf_Len_Pair_t msg_send;

    LRA_USB_Generate_IN_Msg_Stack(USB_IN_CMD_PARSE_ERR, parse_err_msg,
                                  parse_err_msg_len, &msg_send, LRA_FLAG_SET);

    HAL_StatusTypeDef send_status = LRA_USB_Send_Msg(&msg_send, LRA_FLAG_UNSET);

    return parser_status;
  }

  /* reply required region */
  if (!(pmsg.cmd_type & CMD_NR)) {
    /* make msg (stack) */
    if (tx_data_buf == NULL || tx_data_buf_len == 0) {
      // you should never get here
      return PR_RETURN_MSG_TX_UNSET_FAIL | pmsg.cmd_type;
    }

    /* make msg (heap) */
    LRA_USB_Buf_Len_Pair_t msg_send;
    LRA_USB_Generate_IN_Msg_Heap(pmsg.cmd_type, tx_data_buf, tx_data_buf_len,
                                 &msg_send, return_msg_add_eop_flag);
    /* send msg */
    HAL_StatusTypeDef send_status = LRA_USB_Send_Msg(&msg_send, LRA_FLAG_UNSET);
    /* free msg */
    if (msg_send.pbuf != NULL)
      free(msg_send.pbuf);
  }

  /* call callback */
  if (func_callback != NULL)
    func_callback();

  /* PR_OK (0) | pmsg.cmd_type */
  return pmsg.cmd_type;
}

uint8_t LRA_Device_Is_Valid(uint8_t device_index) {
  return (device_index >= LRA_DEVICE_INVALID) ? 0 : 1;
}
