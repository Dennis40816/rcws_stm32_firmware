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
 * desided by the 0th bit of first transfer byte.
 *
 * If you want to write to addr with val 0x05, set your tx_buf[2] = {addr << 1 |
 * 0, 0x05}.
 *
 * If you want to read from addr, set your tx_buf[2] = {addr << 1 | 1, 0}, where
 * 0 is a dummy value (necessary) and a empty rx_buf[2] = {0}. You will get
 * desired data at rx_buf[1].
 *
 * If you still don't understand, please see ADXL355_Read() for more
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

/* public functions */

// set ranges

// calibration

// init

// do2補述

// ID_verifify

// Do self test

// set Filter

// read fifo

/* IO functions */

// read_all

// writeReg

// write

// readReg

// read

/**
 * @brief This functions will fix the problem of lacking iaddr and R bit in
 * buffer, so you can use it if you are lazy to add (iaddr << 1 | 0) at the
 * begin of tx_buf. But it doesn't support read to ADXL355_FIFO.
 *
 * @param hspi
 * @param iaddr
 * @param tx_buf
 * @param rx_buf
 * @param size
 * @param timeout_ms
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef ADXL355_Read(ADXL355_t* const adxl,
                               const ADXL355_Regs_t iaddr,
                               uint8_t* const rx_buf,
                               const int16_t size) {
  if (adxl == NULL || rx_buf == NULL)
    return HAL_ERROR;

  // not support FIFO RW
  if (size > ADXL355_MAX_RW_REG_NUM_ATONCE)
    return HAL_ERROR;

  // TODO: check size maximum for iaddr

  // generate buf correct form for ADXL355
  uint8_t rx_tmp[ADXL355_MAX_RW_REG_NUM_ATONCE + 1] = {0};
  uint8_t tx_tmp[ADXL355_MAX_RW_REG_NUM_ATONCE + 1] = {
      [0] = (iaddr << 1 | ADXL355_SPI_R)};

  HAL_StatusTypeDef ret;

  ret = HAL_SPI_TransmitReceive(adxl->hspi, tx_tmp, rx_tmp, size + 1,
                                adxl->timeout_ms);
  if (ret != HAL_OK)
    return ret;

  // reading finished, copy correct data to user buffer
  memcpy(rx_buf, rx_tmp + 1, size);

  return HAL_OK;
}

/**
 * @brief Use this function for continously reading if you totally know how ADXL355 SPI works
 * 
 * @param adxl 
 * @param tx_buf 
 * @param rx_buf 
 * @param size 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef ADXL355_RawRead(ADXL355_t* const adxl,
                                  uint8_t* const tx_buf,
                                  uint8_t* const rx_buf,
                                  const int16_t size) {
// TODO
                                  }

// readNewest
