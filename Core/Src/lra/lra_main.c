/*
 * lra_main.c
 *
 *  Created on: Feb 8, 2023
 *      Author: Dennis
 */

/* includes */

#include "lra/lra_main.h"
#include "main.h"

/* external variables */

// from main.c
extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim8;

/* public functions */

void LRA_Main_EnterPoint(void) {
  // LED flash *2: enter LRA Main EnterPoint
  LRA_LED_Flash_N(2, 500);

  LRA_Main_System_Init();

  // create error status variable
  uint8_t error = 0;

  /* I2C devices init */

  HAL_StatusTypeDef ret;

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

  // create pairs structs
  TCA_DRV_Pair_t tca_drv_x = {.located_ch = 1, .pDrv = &drv_x, .pTca = &tca};
  TCA_DRV_Pair_t tca_drv_y = {.located_ch = 2, .pDrv = &drv_y, .pTca = &tca};
  TCA_DRV_Pair_t tca_drv_z = {.located_ch = 3, .pDrv = &drv_z, .pTca = &tca};

  // create I2C_Devs struct
  LRA_I2C_Devs_t i2c_devs = {.pair_count = LRA_MOTOR_NUM,
                             .pDevPair = {&tca_drv_x, &tca_drv_y, &tca_drv_z}};

  // create PWM array
  LRA_PWM pwm_x = {
      .htim = &htim2,
      .ch = TIM_CHANNEL_1,
  };

  LRA_PWM pwm_y = {
      .htim = &htim2,
      .ch = TIM_CHANNEL_2,
  };

  LRA_PWM pwm_z = {
      .htim = &htim2,
      .ch = TIM_CHANNEL_3,
  };

  LRA_PWM* pwm_arr[] = {&pwm_x, &pwm_y, &pwm_z};

  // create ADXL355 Instance

  /* I2C devs init */
  ret = LRA_I2C_Devs_Init(&i2c_devs);
  if (ret != HAL_OK)
    error |= (1 << LRA_INIT_ERR_I2C_DEVS);

  /* PWM init */
  for (int i = 0; i < LRA_MOTOR_NUM; i++) {
    ret = Lra_PWM_Init(pwm_arr[i], LRA_DEFAULT_PWM_FREQ);
    if (ret != HAL_OK)
      error |= (1 << LRA_INIT_ERR_PWM);
  }

  /* TODO: MPU6500 init */

  if (ret != HAL_OK)
    error |= (1 << LRA_INIT_ERR_MPU6500);

  /* TODO: ADXL355 init */

  if (ret != HAL_OK)
    error |= (1 << LRA_INIT_ERR_ADXL355);

  // you should check error code here
  error;

  // Flash *2: init end
  LRA_LED_Flash_N(2, 500);

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
   * check enum LRA_PWM_UPDATE_FLAG in lra_main.h
   */
  uint8_t pwm_flag = 0;

  uint32_t pwm_new_freq[LRA_MOTOR_NUM] = {
      LRA_DEFAULT_PWM_FREQ, LRA_DEFAULT_PWM_FREQ, LRA_DEFAULT_PWM_FREQ};
  uint16_t pwm_new_duty[LRA_MOTOR_NUM] = {
      LRA_DEFAULT_PWM_DUTY, LRA_DEFAULT_PWM_DUTY, LRA_DEFAULT_PWM_DUTY};

  // TODO: usb buffers

  // TODO: acc buffers

  while (1) {

    /* Loop update */

    // cmd update flag

    // internal update flag

    /* usb parser */

    // usb receive flag & parser

    // usb tranmit flag

    // --------------------------------

    /* feedback */

    /* calculate new pwm signal */

    /* update pwm singal */

    /* transport data to Rasp */
  }
}

/**
 * @brief
 *
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef LRA_Main_System_Init(void) {
  // System interrupt related
  HAL_TIM_Base_Start_IT(&htim6);

  // DRV init
  DRV2605L_t drvx = {
      .dev_addr = drv2605l_default_addr,
      .timeout_ms = drv2605l_default_timeout_ms,
      .en_pin = 0,
      .en_port = NULL,
      .hi2c = &hi2c1,
  };

  uint8_t drv_buf[DRV2605L_Total_Reg_Num] = {0};
  // combine into lra_i2c_devices later
  DRV2605L_Read_All(&drvx, drv_buf);

  return HAL_OK;
}
