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
#include "lra/lra_dbuf.h"
#include "lra/lra_i2c_devices.h"
#include "lra/lra_pwm.h"
#include "lra/lra_spi_devices.h"
#include "lra/lra_sys_error.h"
#include "lra/lra_timer.h"
#include "lra/lra_usb.h"

// includes parser string operation related headers
#include "string.h"
#include "usbd_cdc_if.h"

/* extern HAL variables */

extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;

/* extern self defined variables */

/* private function declarations */

static uint16_t LRA_USB_Main_Parser();
static uint32_t LRA_USB_Send_ACC(LRA_Acc_Dbuf_t* const dbuf);
static HAL_StatusTypeDef LRA_RCWS_UpdatePWM(
    const LRA_RCWS_PWM_Info_t* const info);
static HAL_StatusTypeDef LRA_DRV2605L_Disable_All();
static HAL_StatusTypeDef LRA_DRV2605L_Enable_All();

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
    .nss_pin = SPI3_NSS_Pin,
    .nss_port = SPI3_NSS_GPIO_Port,
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
    ret = Lra_PWM_Init(pwm_arr[i], LRA_DEFAULT_RESONANT_PWM_FREQ,
                       LRA_DEFAULT_PWM_DUTY);
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

  // CMD, updated by CMD_UPDATE_PWM, used in controller
  float user_freq_cmd[LRA_MOTOR_NUM] = {
      LRA_DEFAULT_USER_FREQ, LRA_DEFAULT_USER_FREQ, LRA_DEFAULT_USER_FREQ};
  uint16_t user_duty_cmd[LRA_MOTOR_NUM] = {
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

  /* LRA driving test */
  // test ok !
  HAL_StatusTypeDef tmp;
  tmp = TCA_DRV_Pair_SwitchCH(&tca_drv_x);
  uint8_t test_data[48];
  tmp = DRV2605L_Read_All(tca_drv_x.pDrv, test_data);
  tmp = DRV2605L_StandbyUnset(tca_drv_x.pDrv);
  tmp = Lra_PWM_Dynamic_Set_Duty(&pwm_x, 500);
#endif

  /* enable ADXL355 */
  ret = ADXL355_Start_Measure(&adxl355);

  while (1) {
    /* Loop update */

    /* usb parser */
    LRA_USB_Main_Parser();

    LRA_Xfer_NAcc_Rbuf2Dbuf(&lra_acc_rb, &lra_acc_dbuf, 100);

    if (LRA_Get_USB_Mode() == LRA_USB_DATA_MODE) {
      /* acc usb dbuf update */
      // 10 is a tmp value

      // usb tranmit flag

      // cmd update flag

      // internal update flag

      // --------------------------------

      /* feedback */

      /* calculate new pwm signal */

      /* update pwm singal */

      /* transport data to Rasp */
      LRA_USB_Send_ACC(&lra_acc_dbuf);
    }
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
  HAL_TIM_Base_Start_IT(&htim7);
  LRA_LED_State_Change(LRA_LED_FLASH);
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

  uint8_t* tx_data_pointer = NULL;
  uint16_t tx_data_len = 0;

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
    uint8_t precheck_msg[PARSER_ERR_BUF_SIZE] = {0};
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
  static uint8_t tx_data_buf[PARSER_MSG_BUF_SIZE];
  uint8_t basic_cmd_type = (pmsg.cmd_type & BASIC_CMD_MASK);
  /**
   * parser preprocess
   *
   * 1. 有固定字串可以比對的，進行比對 (應該只有 init)
   * 2. 需要執行內部程式，執行
   *
   *
   * 以下是每個指令的成功返回格式， {} 代表變數
   * 失敗會返回錯誤類型以及錯誤的指令類別，見 CMD_PARSE_ERR
   *
   * A. CMD_SYS_INFO
   * B. CMD_PARSE_ERR
   * 這兩者是上位機負責處理，不應在 STM32 處理
   *
   * C. CMD_SWITCH_MODE
   * 返回: header + {current mode} + new line
   *
   * D. CMD_INIT
   * 返回: header + init string + new line
   *
   * E. CMD_RESET_DEVICE
   * 返回: header + {device_index} + new line
   */
  switch (basic_cmd_type) {
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

          /* enable / disable DRV2605L */
          if (mode == LRA_USB_CRTL_MODE)
            LRA_DRV2605L_Disable_All();
          else if (mode == LRA_USB_DATA_MODE)
            LRA_DRV2605L_Enable_All();

          tx_data_pointer = tx_data_buf;
          *(tx_data_pointer) = mode;
          *(tx_data_pointer + 1) = '\r';
          *(tx_data_pointer + 2) = '\n';
          tx_data_len = lra_usb_constmsg_in_len[CMD_SWITCH_MODE];
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
        tx_data_pointer = lra_usb_constmsg_in[CMD_INIT];
        tx_data_len = lra_usb_constmsg_in_len[CMD_INIT];
      } else {
        parse_content_error_flag = LRA_FLAG_SET;
      }
      break;

    /**
     * Update device's registers.
     *
     * @param
     * first byte: device
     * second byte: internal address begin
     * third byte: internal address end
     * fourth byte: data start here
     * eop: \r\n
     *
     */
    case CMD_UPDATE_REG: {
      if (LRA_Get_USB_Mode() != LRA_USB_CRTL_MODE) {
        mode_error_flag = LRA_FLAG_SET;
        break;
      }

      uint8_t device_index = *cursor;
      if (!LRA_Device_Is_Valid(device_index)) {
        parse_content_error_flag = LRA_FLAG_SET;
        break;
      }

      // TODO

      break;
    }

    /**
     * Request device's registers.
     *
     * @param
     * first byte: device
     * second byte: internal address begin
     * third byte: internal address end
     * eop: \r\n
     *
     */
    /* This cmd will append eop at the end of data automatically */
    case CMD_GET_REG: {
      if (LRA_Get_USB_Mode() != LRA_USB_CRTL_MODE) {
        mode_error_flag = LRA_FLAG_SET;
        break;
      }

      uint8_t device_index = *cursor;
      if (!LRA_Device_Is_Valid(device_index)) {
        parse_content_error_flag = LRA_FLAG_SET;
        break;
      }

      uint8_t begin_addr = *(cursor + 1);
      uint8_t end_addr = *(cursor + 2);

      // check addr in order
      if (end_addr < begin_addr) {
        parse_content_error_flag = LRA_FLAG_SET;
        break;
      }

      uint16_t addr_len = end_addr - begin_addr + 1;
      return_msg_add_eop_flag = LRA_FLAG_SET;
      tx_data_pointer = tx_data_buf;

      switch (device_index) {
        case LRA_DEVICE_MPU6500:
          // TODO:
          break;

        case LRA_DEVICE_ADXL355: {
          uint8_t addr_overlap = Check_Addr_Overlap(
              begin_addr, end_addr, ADXL355_CONTINUOUS_READ_FORBIDDEN_START,
              ADXL355_CONTINUOUS_READ_FORBIDDEN_END);

          if (addr_overlap) {
            parse_content_error_flag = LRA_FLAG_SET;
            break;
          }

          /* addr upper bound check */
          if (end_addr > ADXL355_Reset) {
            parse_content_error_flag = LRA_FLAG_SET;
            break;
          }

          // read to tx_data_buf (bias cmd(1) + len(2) + device(1) +
          // begin_addr(1) + end_addr(1) = 6 bytes)
          HAL_StatusTypeDef ret =
              ADXL355_LazyRead(&adxl355, begin_addr, tx_data_buf + 6, addr_len);

          if (ret != HAL_OK) {
            internal_operation_error_flag = LRA_FLAG_SET;
            break;
          }

          break;
        }

        case LRA_DEVICE_DRV2605L_X:
        case LRA_DEVICE_DRV2605L_Y:
        case LRA_DEVICE_DRV2605L_Z: {
          /* addr upper bound check */
          if (end_addr > DRV2605L_LRA_Period) {
            parse_content_error_flag = LRA_FLAG_SET;
            break;
          }

          uint8_t drv_dev_index = device_index - LRA_DEVICE_DRV2605L_X;
          const TCA_DRV_Pair_t* const pdrv_dev_pair =
              i2c_devs.pDevPair[drv_dev_index];

          HAL_StatusTypeDef ret;
          ret = TCA_DRV_Pair_Read(pdrv_dev_pair, begin_addr, addr_len,
                                  tx_data_buf + 6);

          if (ret != HAL_OK) {
            internal_operation_error_flag = LRA_FLAG_SET;
            break;
          }

          break;
        }

        default:
          parse_content_error_flag = LRA_FLAG_SET;
          break;
      }

      /* GetReg range - make msg in stack, therefore reserve header placeholder
       */
      *(tx_data_pointer + 3) = device_index;
      *(tx_data_pointer + 4) = begin_addr;
      *(tx_data_pointer + 5) = end_addr;

      // 3 for GetReg range (device_index ...)
      tx_data_len = addr_len + 3;

      break;
    }

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
        case LRA_DEVICE_ALL:
          LRA_USB_SysInfo("Reset all devices: Wait for implement\n");
          break;

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

      tx_data_pointer = tx_data_buf;
      *tx_data_pointer = device_index;
      *(tx_data_pointer + 1) = '\r';
      *(tx_data_pointer + 2) = '\n';
      tx_data_len = lra_usb_constmsg_in_len[CMD_RESET_DEVICE];

      break;
    }

    case CMD_RUN_AUTOCALIBRATE:
      if (LRA_Get_USB_Mode() != LRA_USB_CRTL_MODE) {
        mode_error_flag = LRA_FLAG_SET;
        break;
      }
      /* TODO: call auto calibration */
      break;

    /**
     * CMD_UPDATE_PWM
     * data format:
     * x_pwm_amp (4 bytes) + ',' + x_pwm_freq(4 bytes) + ';'
     * y_pwm_amp (4 bytes) + ',' + y_pwm_freq(4 bytes) + ';'
     * z_pwm_amp (4 bytes) + ',' + z_pwm_freq(4 bytes) + ';'
     * "\r\n"
     *
     * data len: const, 32 bytes
     *
     * return
     * t: float
     * what_we_just_recv(32 bytes)
     *
     * total data: 36 bytes
     */
    case CMD_UPDATE_PWM: {
      // if (LRA_Get_USB_Mode() != LRA_USB_DATA_MODE) {
      //   mode_error_flag = LRA_FLAG_SET;
      //   break;
      // }

      // copy data to avoid rx_buf be modified by other msg
      uint8_t data[64];
      memcpy(data, cursor, pmsg.pdata_len);

      /* parse data into LRA_RCWS_PWM_Info_t */
      LRA_RCWS_PWM_Info_t info;
      HAL_StatusTypeDef parse_result = LRA_Parse_RCWS_PWM_Info(data, &info);

      if (parse_result != HAL_OK) {
        parse_content_error_flag = LRA_FLAG_SET;
        break;
      }

      HAL_StatusTypeDef within_range = LRA_RCWS_PWM_Info_Range_Check(&info);

      if (within_range != HAL_OK) {
        parse_content_error_flag = LRA_FLAG_SET;
        break;
      }

      /* set duty cycle and pwm freq */
      HAL_StatusTypeDef update_state = LRA_RCWS_UpdatePWM(&info);

      if (update_state != HAL_OK) {
        internal_operation_error_flag = LRA_FLAG_SET;
        break;
      }

      float current_time = LRA_Get_Time();

      /* prepare in msg: header left */
      tx_data_pointer = tx_data_buf;
      memcpy(tx_data_pointer + 3, &current_time, sizeof(float));
      memcpy(tx_data_pointer + 3 + sizeof(float), data, pmsg.pdata_len);

      tx_data_len = sizeof(float) + pmsg.pdata_len;

      break;
    }

    case CMD_UPDATE_ACC:
      if (LRA_Get_USB_Mode() != LRA_USB_DATA_MODE) {
        mode_error_flag = LRA_FLAG_SET;
        break;
      }

      /* something goes wrong, you will never get here */
      internal_operation_error_flag = LRA_FLAG_SET;
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

    uint8_t parse_err_msg[PARSER_ERR_BUF_SIZE];
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
    if (tx_data_pointer == NULL || tx_data_len == 0) {
      // you should never get here
      return PR_RETURN_MSG_TX_UNSET_FAIL | pmsg.cmd_type;
    }

    LRA_USB_Buf_Len_Pair_t msg_send;
    HAL_StatusTypeDef send_status;

    /* make msg (stack) */
    if (basic_cmd_type == CMD_GET_REG) {
      LRA_USB_Generate_IN_Msg_Stack(pmsg.cmd_type, tx_data_pointer, tx_data_len,
                                    &msg_send, return_msg_add_eop_flag);
      send_status = LRA_USB_Send_Msg(&msg_send, LRA_FLAG_UNSET);
    } else if (basic_cmd_type == CMD_UPDATE_PWM) {
      LRA_USB_Generate_IN_Msg_Stack(pmsg.cmd_type, tx_data_pointer, tx_data_len,
                                    &msg_send, return_msg_add_eop_flag);
      send_status = LRA_USB_Send_Msg(&msg_send, LRA_FLAG_UNSET);
    }

    /* make msg (heap) */
    else {
      LRA_USB_Generate_IN_Msg_Heap(pmsg.cmd_type, tx_data_pointer, tx_data_len,
                                   &msg_send, return_msg_add_eop_flag);
      /* send msg */
      send_status = LRA_USB_Send_Msg(&msg_send, LRA_FLAG_UNSET);
      /* free msg */
      if (msg_send.pbuf != NULL)
        free(msg_send.pbuf);
    }

    /* check send_status, there's no resend mechanism in current version !! */
    if (send_status != HAL_OK) {
      /* TODO: add USB busy flag here */
    }
  }

  /* call callback */
  if (func_callback != NULL)
    func_callback();

  /* PR_OK (0) | pmsg.cmd_type */
  return pmsg.cmd_type;
}

/**
 * @brief This function becomes a block function if dbuf->data going to xfer is
 * full
 *
 * @param dbuf
 * @return uint32_t
 */
static uint32_t LRA_USB_Send_ACC(LRA_Acc_Dbuf_t* const dbuf) {
  // len check
  uint32_t xfer_len = dbuf->count[dbuf->current_buffer];

  // xfer takes place only if dbuf len >= 200
  if (xfer_len < 200)
    return 0;

  // collect transmit data len and switch dbuf
  LRA_Switch_Dbuf(dbuf);

  uint8_t xfer_buf = 1 - dbuf->current_buffer;
  xfer_len = dbuf->count[xfer_buf];

  // send msg header, 2 for eop len
  uint16_t eop_bias = xfer_len * sizeof(ADXL355_DataSet_t);
  uint16_t data_len = eop_bias + 2;
  uint8_t header[3] = {USB_IN_CMD_UPDATE_ACC, (uint8_t)(data_len >> 8),
                       (uint8_t)data_len};
  USBD_StatusTypeDef ret = CDC_Transmit_FS(header, 3);

  bool eop_combine_flag = (xfer_len < dbuf->size);

  // add eop at the end of dbuf->data that going to xfer
  if (eop_combine_flag) {
    ADXL355_DataSet_t* ptr = dbuf->data[xfer_buf];
    uint8_t* dbuf_end = (uint8_t*)(ptr + xfer_len);
    *dbuf_end = '\r';
    *(dbuf_end + 1) = '\n';
  }

  uint16_t len = eop_combine_flag ? data_len : eop_bias;
  ret = CDC_Transmit_FS((uint8_t*)dbuf->data[xfer_buf], len);

  // send eop seperately, blocks here
  if (!eop_combine_flag) {
    uint8_t eop[2] = {'\r', '\n'};
    ret = CDC_Transmit_FS(eop, 2);
    while (ret != USBD_OK)
      ret = CDC_Transmit_FS(eop, 2);
  }

  return xfer_len;
}

static HAL_StatusTypeDef LRA_DRV2605L_Disable_All() {
  HAL_StatusTypeDef ret;
  ret = TCA_DRV_Pair_SwitchCH(&tca_drv_x);
  ret = DRV2605L_StandbySet(tca_drv_x.pDrv);

  ret = TCA_DRV_Pair_SwitchCH(&tca_drv_y);
  ret = DRV2605L_StandbySet(tca_drv_y.pDrv);

  ret = TCA_DRV_Pair_SwitchCH(&tca_drv_z);
  ret = DRV2605L_StandbySet(tca_drv_z.pDrv);

  return ret;
}

static HAL_StatusTypeDef LRA_DRV2605L_Enable_All() {
  HAL_StatusTypeDef ret;
  ret = TCA_DRV_Pair_SwitchCH(&tca_drv_x);
  ret = DRV2605L_StandbyUnset(tca_drv_x.pDrv);

  ret = TCA_DRV_Pair_SwitchCH(&tca_drv_y);
  ret = DRV2605L_StandbyUnset(tca_drv_y.pDrv);

  ret = TCA_DRV_Pair_SwitchCH(&tca_drv_z);
  ret = DRV2605L_StandbyUnset(tca_drv_z.pDrv);
}

/**
 * @brief Update PWM to STM32.
 *
 * @warning Call LRA_RCWS_PWM_Info_Range_Check() before call this function.
 * @param info
 * @return HAL_StatusTypeDef
 */
static HAL_StatusTypeDef LRA_RCWS_UpdatePWM(
    const LRA_RCWS_PWM_Info_t* const info) {
  uint16_t x_amp = (uint16_t)info->x.amp;
  uint16_t y_amp = (uint16_t)info->y.amp;
  uint16_t z_amp = (uint16_t)info->z.amp;

  Lra_PWM_Dynamic_Set_Duty(&pwm_x, x_amp);
  Lra_PWM_Dynamic_Set_Duty(&pwm_y, y_amp);
  Lra_PWM_Dynamic_Set_Duty(&pwm_z, z_amp);

  // TODO1: add freq parameter update

  return HAL_OK;
}

uint8_t LRA_Device_Is_Valid(uint8_t device_index) {
  return (device_index >= LRA_DEVICE_INVALID) ? 0 : 1;
}
