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
  TCA_DRV_Pair_t tca_drv_x = {
    .located_ch = 1,
    .pDrv = &drv_x,
    .pTca = &tca
  };
  TCA_DRV_Pair_t tca_drv_y = {
    .located_ch = 2,
    .pDrv = &drv_y,
    .pTca = &tca
  };
  TCA_DRV_Pair_t tca_drv_z = {
    .located_ch = 3,
    .pDrv = &drv_z,
    .pTca = &tca
  };

  // create I2C_Devs struct
  LRA_I2C_Devs_t i2c_devs = {
    .pair_count = LRA_MOTOR_NUM,
    .pDevPair = {
      &tca_drv_x,
      &tca_drv_y,
      &tca_drv_z
    }
  };

  // init I2C devs
  ret = LRA_I2C_Devs_Init(&i2c_devs);
  if(ret != HAL_OK)
    error |= (1 << LRA_INIT_ERR_I2C_DEVS);

  /* MPU6500 init */

  if(ret != HAL_OK)
    error |= (1 << LRA_INIT_ERR_MPU6500);

  /* ADXL355 init */

  if(ret != HAL_OK)
    error |= (1 << LRA_INIT_ERR_ADXL355);

  // you should check error code here
  error;

  // Flash *2: init end
  LRA_LED_Flash_N(2, 500);

  #ifdef LRA_TEST
  
  #endif

  while (1) {
    /* feedback */

    /* calculate new pwm signal */

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
