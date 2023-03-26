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

static LRA_USB_Parse_Result_t LRA_USB_Main_Parser();

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

  // create devices' structs
  TCA9546_t tca = {
      .ch = 0,
      .dev_addr = tca9546_default_addr,
      .timeout_ms = tca9546_default_timeout_ms,
      .reset_pin = TCA_NRST_Pin,
      .reset_port = TCA_NRST_GPIO_Port,
      .hi2c = &hi2c1,
  };
  DRV2605L_t drv_x = {
      .dev_addr = drv2605l_default_addr,
      .timeout_ms = drv2605l_default_timeout_ms,
      .en_pin = 0,
      .en_port = NULL,
      .hi2c = &hi2c1,
  };
  DRV2605L_t drv_y = drv_x;
  DRV2605L_t drv_z = drv_x;

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
    if (LRA_USB_Main_Parser() == LRA_USB_PARSE_INIT_OK)
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
    LRA_USB_Print("Init error code: %x", error);
  else
    LRA_USB_Print("Init success\r\n");

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
  LRA_USB_Print("Test started\r\n");

  LRA_USB_Print("Read \r\n");

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

static LRA_USB_Parse_Result_t LRA_USB_Main_Parser() {
  // TODO: get correct buffer if using dual buffers
  uint8_t* const pbuf = lra_usb_rx_user_buf;

  LRA_USB_Msg_t pmsg;

  // LRA_USB_Parse_Precheck() will unset lra_usb_rx_flag no matter the precheck
  // pass or not, so call it before you parse the data
  LRA_USB_Parse_Precheck_t ret = LRA_USB_Parse_Precheck(&pmsg, pbuf);

  if (ret != LRA_USB_PARSE_PRECHECK_OK)
    return LRA_USB_PARSE_FAIL_IN_PRECHECK;

  /* start to parse pdata */
  volatile uint8_t* const cursor = pmsg.pdata;
  switch (pmsg.cmd_type) {
    case LRA_USB_CMD_INIT:
      /* if pdata is verification stirng */
      if (strncmp(pmsg.pdata, LRA_USB_OUT_INIT_STR, pmsg.pdata_len) == 0) {
        // TODO: check the status
        uint8_t usb_state =
            CDC_Transmit_FS((uint8_t*)LRA_USB_IN_INIT_STR, LRA_USB_IN_INIT_DL);
        // change STM32 state
        LRA_Modify_USB_Mode(LRA_USB_CRTL_MODE);
      } else {
        return LRA_USB_PARSE_FAIL_IN_DATA;
      }
      return LRA_USB_PARSE_INIT_OK;

    case LRA_USB_CMD_UPDATE_PWM:
      break;

    case LRA_USB_CMD_UPDATE_REG:
      break;

    case LRA_USB_CMD_RESET_REG:
      break;

    case LRA_USB_CMD_RESET_STM32:
      if (strncmp(pmsg.pdata, LRA_USB_OUT_RESET_STM32_STR, pmsg.pdata_len) ==
          0) {
        if (LRA_Get_USB_Mode() == LRA_USB_CRTL_MODE) {
          uint8_t usb_state = USBD_BUSY;
          while (usb_state == USBD_BUSY) {
            usb_state = CDC_Transmit_FS((uint8_t*)LRA_USB_IN_RESET_STM32_STR,
                                        LRA_USB_IN_RESET_STM32_DL);
          }

          HAL_Delay(1000);

          // reset stm32
          NVIC_SystemReset();
        } else {  // TODO: make it more flexible
          const char* invalid_operation =
              "LRA USB IN Reset STM32 failed: Not in CRTL Mode\r\n";

          CDC_Transmit_FS((uint8_t*)invalid_operation, 50);
        }
      }

      break;

    default:
      return LRA_USB_PARSE_PRECHECK_UNKNOWN;
  }
}
