/*
 * adxl355.c
 *
 *  Created on: Feb 12, 2023
 *      Author: Dennis
 *      Brief: A simple ADXL355 library based on STM32 HAL stm32f4xx_hal.h
 * library and SPI bus.
 */

/* macros */

/**
 * @brief  According to datasheet p.25, the internal address R/W behavior is
 * desided by the 0th bit of first transfer byte. Note that ADXL355 doesn't
 * support duplex readwrite in same SPI operation, that means you should notify
 * ADXL355 Read or Write for following SPI opretaion by set the 0th bit of first
 * byte in tx_buf to correct state.
 *
 * If you want to write to addr with val 0x05, set your tx_buf[2] = {addr << 1 |
 * 0, 0x05}. The 0 in the  first element means SPI write in ADXL355.
 *
 * If you want to read from addr, set your tx_buf[2] = {addr << 1 | 1, 0}, where
 * 0 is a dummy value (necessary) and a empty rx_buf[2] = {0}. You will get
 * desired data at rx_buf[1].
 *
 * If you still don't understand, please see ADXL355_LazyRead() for more
 * details.
 */
#define ADXL355_SPI_W (0x00)
#define ADXL355_SPI_R (0x01)

// not including FIFO
#define ADXL355_MAX_RW_REG_NUM_ATONCE 18

/* includes */

#include "devices/adxl355.h"
#include <string.h>
#include "util/util_spi.h"

/* extern variable assignment */

const uint8_t adxl355_1st_regnum = 17;
const uint8_t adxl355_2nd_regnum = 18;

/* static functions declarations */

static void ADXL355_Software_NSS_Enable(ADXL355_t* const pAdxl);
static void ADXL355_Software_NSS_Disable(ADXL355_t* const pAdxl);
static HAL_StatusTypeDef ADXL355_UserConfig(ADXL355_t* const pAdxl);

/* public functions */

/**
 * @brief Read register ADXL355_Range and update the read val to pAdxl->range
 *
 * @param pAdxl
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_GetRange(ADXL355_t* const pAdxl) {
  uint8_t val;
  HAL_StatusTypeDef ret = ADXL355_ReadReg(pAdxl, ADXL355_Range, &val);

  if (ret != HAL_OK)
    return ret;

  pAdxl->range = val & 0b11;  // mask other bits except [0:1]

  return HAL_OK;
}

/**
 * @brief Update register ADXL355_Range with the val corresponding to
 * new_range.
 *
 * @param pAdxl
 * @param new_range
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_SetRange(ADXL355_t* const pAdxl,
                                   ADXL355_Ranges_t new_range) {
  uint8_t old_val;
  HAL_StatusTypeDef ret = ADXL355_ReadReg(pAdxl, ADXL355_Range, &old_val);

  if (ret != HAL_OK)
    return ret;

  if ((old_val & 0b11) == new_range)
    return HAL_OK;

  uint8_t new_val = ((old_val & (~0b11)) | new_range);
  ret = ADXL355_WriteReg(pAdxl, ADXL355_Range, new_val);

  if (ret != HAL_OK)
    return ret;

  // update pAdxl->range
  pAdxl->range = new_range;
  return HAL_OK;
}

/**
 * @brief
 *
 * @param pAdxl
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Calibrate(ADXL355_t* const pAdxl) {
  // TODO
}

/**
 * @brief Verify pAdxl is valid or not. Note that this function will modify
 * timeout_ms if == 0. You should create a new instance of ADXL355_t, assign the
 * parameters and then pass it's address.
 *
 * @param pAdxl
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Init(ADXL355_t* const pAdxl) {
  // verify pAdxl is valid
  if (pAdxl->timeout_ms == 0)
    pAdxl->timeout_ms = 1;

  // verify device part id
  if (ADXL355_ID_Verify(pAdxl) <= 0)
    return HAL_ERROR;

  // range check for pAdxl->range
  if (!(pAdxl->range == acc_2g && pAdxl->range == acc_4g &&
        pAdxl->range == acc_8g))
    return HAL_ERROR;

  // set range
  HAL_StatusTypeDef ret = ADXL355_SetRange(pAdxl, pAdxl->range);
  if (ret != HAL_OK)
    return ret;

  // calibrate
  ret = ADXL355_Calibrate(pAdxl);
  if (ret != HAL_OK)
    return ret;

  return ADXL355_UserConfig(pAdxl);
}

/**
 * @brief Return real value of range represented by pAdxl->range
 *
 * @param pAdxl
 * @return float
 */
float ADXL355_GetRangeCache(const ADXL355_t* const pAdxl) {
  float range_ = 0.0;

  switch (pAdxl->range) {
    case acc_2g:
      range_ = 4.096f;  // 2.048 * 2
      break;
    case acc_4g:
      range_ = 8.192f;  // 4.096 * 2
      break;
    case acc_8g:
      range_ = 16.384f;  // 8.192 * 2
      break;
    default:
      break;
  }

  return range_;
}

/**
 * @brief Parse offset to ADXL355_DataSet_t (in float format)
 *
 * @param pAdxl
 * @param pOffset
 * @param pResult
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_ParseOffset(ADXL355_t* const pAdxl,
                                      const uint8_t* const pOffset,
                                      ADXL355_DataSet_t* const pResult) {
  if (pOffset == NULL || pResult == NULL)
    return HAL_ERROR;

  // a 16 bits two complement -> just use int16_t to parse
  int16_t offset[3] = {
      (pOffset[0]) << 8 | pOffset[1],
      (pOffset[2]) << 8 | pOffset[3],
      (pOffset[4]) << 8 | pOffset[5],
  };

  // get cache range in ADXL355_t
  float range_ = ADXL355_GetRangeCache(pAdxl);

  if (range_ == 0.0)
    return HAL_ERROR;

  // 2^16 = 65536
  for (int i = 0; i < 3; i++) {
    pResult->data[i] = offset[i] * (float)1 / 65536 * range_;
  }

  return HAL_OK;
}

/**
 * @brief Parse data by two complement. pData should be a pointer or a array
 * contains 9 bytes [XData3, XData2, ..., ZData2, ZData1]
 *
 * @param pData
 * @param pResult required length == 9 (3 axes)
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_ParseDataSet(ADXL355_t* const pAdxl,
                                       uint8_t* pData,
                                       ADXL355_DataSet_t* pResult) {
  if (pData == NULL)
    return HAL_ERROR;

  static uint32_t u32_data[3];  // x, y, z
  static int32_t i32_data[3];
#define MASK_20 ((1 << 20) - 1)

  u32_data[0] = (pData[0] << 16) | (pData[1] << 8) | (pData[2]);
  u32_data[1] = (pData[3] << 16) | (pData[4] << 8) | (pData[5]);
  u32_data[2] = (pData[6] << 16) | (pData[7] << 8) | (pData[8]);

  // convert to float
  float range_ = ADXL355_GetCacheRange(pAdxl);
  const static uint32_t adc_step_num = 1048576;  // 2^20

  for (int i = 0; i < 3; i++) {
    i32_data[i] = (u32_data[i] & (1 << 19))  // if 19th bit is 1, then inverse
                      ? u32_data[i] | (~MASK_20)
                      : u32_data[i];
    pResult->data[i] = ((float)i32_data[i]) * (1.0 / adc_step_num) * range_;
  }
  return HAL_OK;

#undef MASK_20
}

/**
 * @brief Verify Part ID is 0xED (is ADXL355). If the part ID is 0xED return 1,
 * else return 0. If failure happens return -1.
 *
 * @param pAdxl
 * @return int8_t
 */
int8_t ADXL355_ID_Verify(ADXL355_t* const pAdxl) {
  uint8_t val;
  HAL_StatusTypeDef ret = ADXL355_ReadReg(pAdxl, ADXL355_PARTID, &val);

  if (ret != HAL_OK)
    return -1;

  return (val == 0xED) ? 1 : 0;
}

/**
 * @brief Do selftest with st internal force applying on ADXL355. See
 * ADXL355_SELF_TEST for details.
 *
 * @param pAdxl
 * @param st set 1 or 2
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_SelfTest(ADXL355_t* const pAdxl, uint8_t st) {
  if (st != 1 && st != 2)
    return HAL_ERROR;

  // TODO
}

/**
 * @brief Start to measure data
 *
 * @param pAdxl
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Start_Measure(ADXL355_t* const pAdxl) {
  uint8_t val;
  HAL_StatusTypeDef ret = ADXL355_ReadReg(pAdxl, ADXL355_POWER_CTL, &val);

  if (ret != HAL_OK)
    return ret;

  val &= ~0b1;

  return ADXL355_WriteReg(pAdxl, ADXL355_POWER_CTL, val);
}

/**
 * @brief Stop to measure data
 *
 * @param pAdxl
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Stop_Measure(ADXL355_t* const pAdxl) {
  uint8_t val;
  HAL_StatusTypeDef ret = ADXL355_ReadReg(pAdxl, ADXL355_POWER_CTL, &val);

  if (ret != HAL_OK)
    return ret;

  val |= 0b1;

  return ADXL355_WriteReg(pAdxl, ADXL355_POWER_CTL, val);
}

/**
 * @brief Set acceleration offset (from 0x1E to 0x23) from pOffset
 *
 * @param pAdxl
 * @param pOffset required len == 6
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Write_Offset(ADXL355_t* const pAdxl,
                                       uint8_t* const pOffset) {
  return ADXL355_LazyWrite(pAdxl, ADXL355_OFFSET_X_H, pOffset, 6);
}

/**
 * @brief Read offset registers to pResult
 *
 * @param pAdxl
 * @param pResult required 6 bytes
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Read_Offset(ADXL355_t* const pAdxl,
                                      ADXL355_DataSet_t* const pResult) {
  uint8_t tmp[6];
  HAL_StatusTypeDef ret = ADXL355_LazyRead(pAdxl, ADXL355_OFFSET_X_H, tmp, 6);

  if (ret != HAL_OK)
    return ret;

  return ADXL355_ParseOffset(pAdxl, tmp, pResult);
}

/**
 * @brief All of functions below require appropriate length of buffer to store
 * read data, which means user should prepare a buffer array with correct
 * size, not only send in a pointer.
 *
 * @param pAdxl
 * @param pResult
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Read_Temp(ADXL355_t* const pAdxl, float* pResult) {
  if (pResult == NULL || pAdxl == NULL || pAdxl->hspi == NULL)
    return HAL_ERROR;

  uint8_t tx_buf[3] = {(ADXL355_TEMP2 << 1 | ADXL355_SPI_R)};
  uint8_t rx_buf[3] = {0};

  ADXL355_Software_NSS_Enable(pAdxl);

  HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(pAdxl->hspi, tx_buf, rx_buf,
                                                  3, pAdxl->timeout_ms);

  ADXL355_Software_NSS_Disable(pAdxl);

  if (ret != HAL_OK)
    return ret;

  uint16_t val = ((rx_buf[1] & 0b1111) << 8 | rx_buf[2]);

  // parse it according to datasheet p.33
  float tmp_f;
  if (pAdxl->temp_intercept_lsb == 0 || pAdxl->temp_slope == 0.0) {
    // use default uncompensated value
    tmp_f = -9.05 * (val - 1852) + 25.0;
  } else {
    tmp_f = pAdxl->temp_slope * (val - pAdxl->temp_intercept_lsb) +
            pAdxl->temp_intercept_Celsius;
  }

  // tmp_f check, should between -40 ~ 125 Celsius
  if (tmp_f < -40.0 || tmp_f > 125.0)
    return HAL_ERROR;

  *pResult = tmp_f;
  return HAL_OK;
}

/**
 * @brief
 *
 * @param pAdxl
 * @param rx_buf required len
 * @param num
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Read_FIFO(ADXL355_t* const pAdxl,
                                    uint8_t* const rx_buf,
                                    uint8_t num) {
  // TODO:
}

/**
 * @brief Read newest data starts from ADXL355_XDATA3 to ADXL355_ZDATA1.
 *
 * @param pAdxl
 * @param rx_buf
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Read_NewestData(ADXL355_t* const pAdxl,
                                          uint8_t* const rx_buf) {
  if (pAdxl == NULL || rx_buf == NULL || pAdxl->hspi == NULL)
    return HAL_ERROR;

  // This function will be call frequently, so define as static
  static const uint8_t tx_tmp[10] = {(ADXL355_XDATA3 << 1 | ADXL355_SPI_R)};
  static uint8_t rx_tmp[10] = {0};

  ADXL355_Software_NSS_Enable(pAdxl);

  HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(pAdxl->hspi, tx_tmp, rx_tmp,
                                                  10, pAdxl->timeout_ms);

  ADXL355_Software_NSS_Disable(pAdxl);

  if (ret != HAL_OK)
    return ret;

  memcpy(rx_buf, rx_tmp + 1, 9);
  return HAL_OK;
}

/**
 * @brief Read all available resigers value. Registers can be seperated into 2
 * parts.The first part (Read only) is from 0x00 to 0x10 (total 17). The
 * second part is from 0x1E to 0x2F (total 18). Note that ADXL355_FIFO_DATA is
 * not included.
 *
 * Therefore, user should prepare an uint8_t rx_buf[17+18] for this function.
 *
 * @param pAdxl
 * @param rx_buf required length > 35
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Read_All(ADXL355_t* const pAdxl,
                                   uint8_t* const rx_buf) {
  if (pAdxl == NULL || pAdxl->hspi == NULL || rx_buf == NULL)
    return HAL_ERROR;

  uint8_t rx_tmp[ADXL355_MAX_RW_REG_NUM_ATONCE + 1] = {0};
  uint8_t tx_tmp[ADXL355_MAX_RW_REG_NUM_ATONCE + 1] = {
      [0] = (ADXL355_DEVID_AD << 1 | ADXL355_SPI_R)};

  ADXL355_Software_NSS_Enable(pAdxl);

  // read 1st region
  HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(
      pAdxl->hspi, tx_tmp, rx_tmp, adxl355_1st_regnum + 1, pAdxl->timeout_ms);

  if (ret != HAL_OK)
    return ret;

  memcpy(rx_buf, rx_tmp + 1, adxl355_1st_regnum);

  // read 2nd region
  tx_tmp[0] = (_ADXL355_WRITABLE_START << 1 | ADXL355_SPI_R);
  ret = HAL_SPI_TransmitReceive(pAdxl->hspi, tx_tmp, rx_tmp,
                                adxl355_2nd_regnum + 1, pAdxl->timeout_ms);

  ADXL355_Software_NSS_Disable(pAdxl);

  if (ret != HAL_OK)
    return ret;

  memcpy(rx_buf + adxl355_1st_regnum, rx_tmp + 1, adxl355_2nd_regnum);
  return HAL_OK;
}

/* IO functions */

/**
 * @brief Transmit one byte data to target iaddr.
 *
 * @param pAdxl
 * @param iaddr
 * @param val
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_WriteReg(ADXL355_t* const pAdxl,
                                   const ADXL355_Regs_t iaddr,
                                   uint8_t val) {
  if (pAdxl == NULL || pAdxl->hspi == NULL)
    return HAL_ERROR;

  // iaddr check
  if (iaddr > ADXL355_Reset)
    return HAL_ERROR;

  uint8_t rx_tmp[2] = {0};
  uint8_t tx_tmp[2] = {[0] = (iaddr << 1 | ADXL355_SPI_W), val};

  ADXL355_Software_NSS_Enable(pAdxl);

  HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(pAdxl->hspi, tx_tmp, rx_tmp,
                                                  2, pAdxl->timeout_ms);

  ADXL355_Software_NSS_Disable(pAdxl);

  return ret;
}

/**
 * @brief Read one byte data from target iaddr and update to pResult if SPI
 * read return HAL_OK.
 *
 * @param pAdxl
 * @param iaddr
 * @param pResult
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_ReadReg(ADXL355_t* const pAdxl,
                                  const ADXL355_Regs_t iaddr,
                                  uint8_t* pResult) {
  if (pAdxl == NULL || pResult == NULL || pAdxl->hspi == NULL)
    return HAL_ERROR;

  // iaddr check
  if (iaddr > ADXL355_Reset)
    return HAL_ERROR;

  uint8_t rx_tmp[2] = {0};
  uint8_t tx_tmp[2] = {[0] = (iaddr << 1 | ADXL355_SPI_R)};

  ADXL355_Software_NSS_Enable(pAdxl);

  HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(pAdxl->hspi, tx_tmp, rx_tmp,
                                                  2, pAdxl->timeout_ms);

  ADXL355_Software_NSS_Disable(pAdxl);

  if (ret == HAL_OK)
    *pResult = rx_tmp[1];

  return ret;
}

/**
 * @brief Simiar to ADXL355_LazyRead, this function enable user to separate
 * iaddr from tx_buf.
 *
 * @param pAdxl
 * @param iaddr
 * @param tx_buf
 * @param data_size
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_LazyWrite(ADXL355_t* const pAdxl,
                                    const ADXL355_Regs_t iaddr,
                                    uint8_t* const tx_buf,
                                    const int16_t data_size) {
  if (pAdxl == NULL || tx_buf == NULL || pAdxl->hspi == NULL)
    return HAL_ERROR;

  if (data_size > ADXL355_MAX_RW_REG_NUM_ATONCE)
    return HAL_ERROR;

  if (iaddr > ADXL355_Reset)
    return HAL_ERROR;

  // generate buf correct form for ADXL355
  uint8_t rx_tmp[ADXL355_MAX_RW_REG_NUM_ATONCE + 1] = {0};
  uint8_t tx_tmp[ADXL355_MAX_RW_REG_NUM_ATONCE + 1] = {
      [0] = (iaddr << 1 | ADXL355_SPI_W)};

  memcpy(tx_tmp + 1, tx_buf, data_size);

  ADXL355_Software_NSS_Enable(pAdxl);

  HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(
      pAdxl->hspi, tx_tmp, rx_tmp, data_size + 1, pAdxl->timeout_ms);

  ADXL355_Software_NSS_Disable(pAdxl);

  return ret;
}

/**
 * @brief This function will fix the problem of lacking iaddr and R bit at the
 * begining of the buffer, so you can use it if you are lazy to add (iaddr <<
 * 1 | 0) at the begin of tx_buf. But it doesn't support read from
 * ADXL355_FIFO_DATA.
 *
 * @param hspi
 * @param iaddr
 * @param rx_buf required length == data_size
 * @param data_size pure data length, not including iaddr byte
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_LazyRead(ADXL355_t* const pAdxl,
                                   const ADXL355_Regs_t iaddr,
                                   uint8_t* const rx_buf,
                                   const int16_t data_size) {
  if (pAdxl == NULL || rx_buf == NULL || pAdxl->hspi == NULL)
    return HAL_ERROR;

  // not support FIFO RW
  if (data_size > ADXL355_MAX_RW_REG_NUM_ATONCE)
    return HAL_ERROR;

  if (iaddr > ADXL355_Reset)
    return HAL_ERROR;

  // TODO: check size maximum for iaddr

  // generate buf correct form for ADXL355
  uint8_t rx_tmp[ADXL355_MAX_RW_REG_NUM_ATONCE + 1] = {0};
  uint8_t tx_tmp[ADXL355_MAX_RW_REG_NUM_ATONCE + 1] = {
      [0] = (iaddr << 1 | ADXL355_SPI_R)};

  ADXL355_Software_NSS_Enable(pAdxl);

  HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(
      pAdxl->hspi, tx_tmp, rx_tmp, data_size + 1, pAdxl->timeout_ms);

  ADXL355_Software_NSS_Disable(pAdxl);

  if (ret != HAL_OK)
    return ret;

  // reading finished, copy correct data to user buffer
  memcpy(rx_buf, rx_tmp + 1, data_size);

  return HAL_OK;
}

/**
 * @brief This function use the first byte of tx_buf directly to decide the
 * SPI tx/rx behavior. Therefore, you need to read and understand datasheet
 * p.25 to set the tx_buf to the correct form. Note that rx_buf size should be
 * size + 1, and you should ignore the first byte after "read" operation
 * (should be 0 at 0th byte).
 *
 * @details Though the efficiency is higher than LazyRead or LazyWrite, you
 * should take the responsibility of wrong operation.
 * @param pAdxl
 * @param tx_buf
 * @param rx_buf required length == total_size
 * @param total_size including iaddr byte
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_RW(ADXL355_t* const pAdxl,
                             uint8_t* const tx_buf,
                             uint8_t* const rx_buf,
                             const int16_t total_size) {
  if (pAdxl == NULL || tx_buf == NULL || rx_buf == NULL || pAdxl->hspi == NULL)
    return HAL_ERROR;

  ADXL355_Software_NSS_Enable(pAdxl);

  HAL_StatusTypeDef ret = HAL_SPI_TransmitReceive(
      pAdxl->hspi, tx_buf, rx_buf, total_size, pAdxl->timeout_ms);

  ADXL355_Software_NSS_Disable(pAdxl);

  return ret;
}

/* private functions */

/**
 * @brief Pull low NSS pin if using software NSS
 *
 * @param pAdxl
 */
static void ADXL355_Software_NSS_Enable(ADXL355_t* const pAdxl) {
  if (pAdxl->nss_port == NULL)
    return;

  HAL_GPIO_WritePin(pAdxl->nss_port, pAdxl->nss_pin, GPIO_PIN_RESET);
}

/**
 * @brief Pull high NSS pin if using software NSS
 *
 * @param pAdxl
 */
static void ADXL355_Software_NSS_Disable(ADXL355_t* const pAdxl) {
  if (pAdxl->nss_port == NULL)
    return;

  HAL_GPIO_WritePin(pAdxl->nss_port, pAdxl->nss_pin, GPIO_PIN_SET);
}

/**
 * @brief User defined, left empty if not use.
 *
 * @param pAdxl
 * @return HAL_StatusTypeDef
 */
static HAL_StatusTypeDef ADXL355_UserConfig(ADXL355_t* const pAdxl) {
  return HAL_OK;
}
