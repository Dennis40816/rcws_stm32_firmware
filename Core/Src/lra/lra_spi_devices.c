/*
 * lra_spi_devices.c
 *
 *  Created on: Mar 22, 2023
 *      Author: Dennis Liu
 */

/* includes */

#include "lra/lra_spi_devices.h"

/* extern variables definitions */

uint8_t lra_acc_buf1[LRA_ACC_BUFFER_SIZE] = {0};
uint8_t lra_acc_buf2[LRA_ACC_BUFFER_SIZE] = {0};
LRA_DualBuf_t lra_acc_dbuf;

/* public functions */

HAL_StatusTypeDef ADXL355_Read_NewestData_DualBuf(ADXL355_t* const pAdxl,
                                                  LRA_DualBuf_t* const dbuf) {
  if (pAdxl == NULL || dbuf == NULL || pAdxl->hspi == NULL)
    return HAL_ERROR;

  // 10 for 9 data bytes + 1 dummy byte
  static const uint8_t tx_tmp[10] = {(ADXL355_XDATA3 << 1 | ADXL355_SPI_R)};
  static uint8_t rx_tmp[10] = {0};

  ADXL355_Software_NSS_Enable(pAdxl);

  HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(pAdxl->hspi, tx_tmp, rx_tmp,
                                                  10, pAdxl->timeout_ms);

  ADXL355_Software_NSS_Disable(pAdxl);

  if (ret != HAL_OK)
    return ret;

  return LRA_Write_To_DualBuf(dbuf, rx_tmp + 1, 9);
}
