/*
 * drv2605l.c
 *
 *  Created on: Feb 9, 2023
 *      Author: Dennis
 */

/* includes */

#include "devices/drv2605l.h"

/* exported variables */
const uint8_t drv2605l_default_addr = 0x5A;
const uint16_t drv2605l_default_timeout_ms = STM32_I2C_MIN_TIMEOUT_MS;

/* static function declaration */

static HAL_StatusTypeDef DRV_UnsafeRead(const DRV2605L_t* const pDrv,
                                        const DRV_Regs iaddr,
                                        const uint16_t len,
                                        uint8_t* const pData);

static HAL_StatusTypeDef DRV_UnsafeWrite(const DRV2605L_t* const pDrv,
                                         const DRV_Regs iaddr,
                                         const uint16_t len,
                                         uint8_t* const pData);

static HAL_StatusTypeDef DRV_UnsafeReadReg(const DRV2605L_t* const pDrv,
                                           const uint16_t iaddr,
                                           uint8_t* pData);

static HAL_StatusTypeDef DRV_UnsafeWriteReg(const DRV2605L_t* const pDrv,
                                            const uint16_t iaddr,
                                            uint8_t data);

/* public functions */

/**
 * @brief Write to Mode register @7th bit to reset drv2605l device to default
 * state
 *
 * @param pDrv
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_SoftReset(const DRV2605L_t* const pDrv) {
  if (pDrv == NULL || pDrv->hi2c == NULL)
    return HAL_ERROR;

  return DRV_UnsafeWriteReg(pDrv, Mode, 0x01 << 7);
}

/**
 * @brief Set standby bit (set it to 1) to make DRV2605L get into standby mode
 *
 * @details Standby bit is at register Mode, 6th bit
 * @param pDrv
 * @param bStandby
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_StandbySet(const DRV2605L_t* const pDrv) {
  if (pDrv == NULL || pDrv->hi2c == NULL)
    return HAL_ERROR;

  // read Mode register first
  uint8_t val;
  HAL_StatusTypeDef ret = DRV_UnsafeRead(pDrv, Mode, 1, &val);
  if (ret != HAL_OK)
    return ret;

  // set standby bit always to 1
  // uint8_t new_val = (1 << 6) | val;
  val |= 1 << 6;

  // update to register
  return DRV_UnsafeWrite(pDrv, Mode, 1, &val);
}

/**
 * @brief Unset standby bit (set it to 0) to make DRV2605L get out from standby
 * mode
 *
 * @param pDrv
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_StandbyUnset(const DRV2605L_t* const pDrv) {
  if (pDrv == NULL || pDrv->hi2c == NULL)
    return HAL_ERROR;

  // read Mode register first
  uint8_t val;
  HAL_StatusTypeDef ret = DRV_UnsafeRead(pDrv, Mode, 1, &val);
  if (ret != HAL_OK)
    return ret;

  // set standby bit always to 0, (~(1 << 6)) == 0b1011 1111
  // uint8_t new_val = (~(1 << 6)) & val;
  val &= ~(1 << 6);

  // update to register
  return DRV_UnsafeWrite(pDrv, Mode, 1, &val);
}

/**
 * @brief Set Go bit to 1
 *
 * @param drv
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_GoSet(const DRV2605L_t* const pDrv) {
  if (pDrv == NULL || pDrv->hi2c == NULL)
    return HAL_ERROR;

  uint8_t val = 0x01;

  return DRV_UnsafeWrite(pDrv, Go, 1, &val);
}

/**
 * @brief Set Go bit to 0
 *
 * @param pDrv
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_GoUnset(const DRV2605L_t* const pDrv) {
  if (pDrv == NULL || pDrv->hi2c == NULL)
    return HAL_ERROR;

  uint8_t val = 0x00;

  return DRV_UnsafeWrite(pDrv, Go, 1, &val);
}

/**
 * @brief Run calibration with current parameters and assign result to pResult.
 * The result is valid only if the return value of the function is HAL_OK. Make
 * sure that you know this function will block by HAL_Delay(1200) to wait for
 * auto calibration done.
 *
 * @details 在使用時需要注意返回值是否是 HAL_OK，當 HAL_OK 才代表結果可信。檢查
 * bit 3 可以知道結果
 * @param pDrv
 * @param pResult
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_Run_AutoCalibration(const DRV2605L_t* const pDrv,
                                          uint8_t* pResult) {
  // Check
  if (pDrv == NULL || pDrv->hi2c == NULL || pResult == NULL)
    return HAL_ERROR;

  // Copy Go and Mode Reg
  uint8_t tmp_Go;
  HAL_StatusTypeDef ret = DRV_UnsafeReadReg(pDrv, Go, &tmp_Go);
  if (ret != HAL_OK)
    return ret;

  uint8_t tmp_Mode;
  ret = DRV_UnsafeReadReg(pDrv, Mode, &tmp_Mode);
  if (ret != HAL_OK)
    return ret;

  // Make the device stop anyway
  ret = DRV_UnsafeWriteReg(pDrv, Go, 0x00);
  if (ret != HAL_OK)
    return ret;

  // ret = DRV_UnsafeWriteReg(pDrv, Mode, tmp_Mode | (1 << 6));
  // if (ret != HAL_OK)
  //   return ret;

  // Write 0x07 to register: Mode, which makes the device get ready and go into
  // auto calibration mode.
  ret = DRV_WriteReg(pDrv, Mode, 0x07);
  if (ret != HAL_OK)
    return ret;

  ret = DRV_UnsafeWriteReg(pDrv, Go, 0x01);
  if (ret != HAL_OK)
    return ret;

  // wait for auto calibration to complete, max 1.2 second by datasheet
  HAL_Delay(1200);

  // Get the result
  ret = DRV_UnsafeReadReg(pDrv, Status, pResult);
  if (ret != HAL_OK)
    return ret;

  // Resume Mode and Go register
  ret = DRV_UnsafeWriteReg(pDrv, Mode, tmp_Mode);
  if (ret != HAL_OK)
    return ret;

  ret = DRV_UnsafeWriteReg(pDrv, Go, tmp_Go);
  if (ret != HAL_OK)
    return ret;

  return HAL_OK;
}

/**
 * @brief Set DRV2605L to PWM control mode with standby bit set. Activate
 * actuator by calling DRV_StandbyUnset(). Note that if you want to stop driving
 * actuator, you must set STANDBY bit to 1. DRV_GoUnset() [Go bit] is useless in
 * PWM mode. Call DRV_StandbySet() instead.
 *
 * @details See datasheet p.27 and p.31
 * @param pDrv
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_SetMode_PWM(const DRV2605L_t* const pDrv) {
  if (pDrv == NULL || pDrv->hi2c == NULL)
    return HAL_ERROR;

  // write 0x03 | 1<< 6 to Mode register (standby)
  HAL_StatusTypeDef ret = DRV_UnsafeWriteReg(pDrv, Mode, 0x03 | (1 << 6));
  if (ret != HAL_OK)
    return ret;

  // set N_PWM_ANALOG bit (Control 3, 1.st bit) to 0
  uint8_t tmp_Control3;
  ret = DRV_UnsafeReadReg(pDrv, Control3, &tmp_Control3);
  if (ret != HAL_OK)
    return ret;

  ret = DRV_UnsafeWriteReg(pDrv, Control3, tmp_Control3 & ~(1 << 1));
  if (ret != HAL_OK)
    return ret;

  return HAL_OK;
}

/* IO functions */

/**
 * @brief Read all registers from DRV2605L
 *
 * @param pDrv
 * @param pData
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_Read_All(const DRV2605L_t* const pDrv, uint8_t* pData) {
  if (pDrv == NULL || pDrv->hi2c == NULL || pData == NULL) 
    return HAL_ERROR;

  return DRV_UnsafeRead(pDrv, Status, DRV_Total_Reg_Num, pData);
}

/**
 * @brief Read one byte data without any check. Internal usage only.
 *
 * @param pDrv
 * @param iaddr
 * @param pData
 * @return HAL_StatusTypeDef
 */
static HAL_StatusTypeDef DRV_UnsafeReadReg(const DRV2605L_t* const pDrv,
                                           const uint16_t iaddr,
                                           uint8_t* pData) {
  return DRV_UnsafeRead(pDrv, iaddr, 1, pData);
}

/**
 * @brief Read one byte data to pData from iaddr
 *
 * @param pDrv
 * @param iaddr DRV_Regs
 * @param pData
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_ReadReg(const DRV2605L_t* const pDrv,
                              const uint16_t iaddr,
                              uint8_t* pData) {
  if (pDrv == NULL || pDrv->hi2c == NULL || pData == NULL) 
    return HAL_ERROR;

  return DRV_UnsafeRead(pDrv, iaddr, 1, pData);
}

/**
 * @brief Write one byte data without any check. Internal usage only.
 *
 * @param pDrv
 * @param iaddr
 * @param data
 * @return HAL_StatusTypeDef
 */
static HAL_StatusTypeDef DRV_UnsafeWriteReg(const DRV2605L_t* const pDrv,
                                            const uint16_t iaddr,
                                            uint8_t data) {
  return DRV_UnsafeWrite(pDrv, iaddr, 1, &data);
}

/**
 * @brief Write one byte data to iaddr
 *
 * @param pDrv
 * @param iaddr
 * @param data
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_WriteReg(const DRV2605L_t* const pDrv,
                               const uint16_t iaddr,
                               uint8_t data) {
  if (pDrv == NULL || pDrv->hi2c == NULL)
    return HAL_ERROR;

  return DRV_UnsafeWrite(pDrv, iaddr, 1, &data);
}

/**
 * @brief 未進行任何保護對 DRV device 進行寫入
 *
 * @param pDrv
 * @param iaddr
 * @param len
 * @param pData
 * @return HAL_StatusTypeDef
 */
static HAL_StatusTypeDef DRV_UnsafeWrite(const DRV2605L_t* const pDrv,
                                         const DRV_Regs iaddr,
                                         const uint16_t len,
                                         uint8_t* const pData) {
  return HAL_I2C_Mem_Write(pDrv->hi2c, (pDrv->dev_addr) << I2C_W, iaddr,
                           DRV2605L_MEMADDSIZE, pData, len, pDrv->timeout_ms);
}

/**
 * @brief 針對 DRV2605L 進行多字節寫入
 *
 * @param pDrv
 * @param iaddr
 * @param len
 * @param pData
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_Write(const DRV2605L_t* const pDrv,
                            const DRV_Regs iaddr,
                            uint16_t len,
                            uint8_t* const pData) {
  if (pDrv == NULL)
    return HAL_ERROR;

  if (pDrv->hi2c == NULL)
    return HAL_ERROR;

  if (pData == NULL)
    return HAL_ERROR;

  if (iaddr > LRA_Period)
    return HAL_ERROR;

  if (len > (DRV_Total_Reg_Num - iaddr))
    len = DRV_Total_Reg_Num - iaddr;

  return DRV_UnsafeWrite(pDrv, iaddr, len, pData);
}

/**
 * @brief 放棄所有檢查，直接讀取後放回 buf data，應該寫成 【static】
 * 確保僅內部調用
 *
 * @param pDrv
 * @param iaddr
 * @param len
 * @param pData
 * @return HAL_StatusTypeDef
 */
static HAL_StatusTypeDef DRV_UnsafeRead(const DRV2605L_t* const pDrv,
                                        const DRV_Regs iaddr,
                                        uint16_t len,
                                        uint8_t* const pData) {
  return HAL_I2C_Mem_Read(pDrv->hi2c, (pDrv->dev_addr) << I2C_R, iaddr,
                          DRV2605L_MEMADDSIZE, pData, len, pDrv->timeout_ms);
}

/**
 * @brief 確認所有指標皆有效，且確保讀取 iaddr (起始點) + len 不超過最後一個
 * register。
 *
 * @param pDrv pointer to a DRV2605L_t structure
 * @param iaddr start address of internal memory
 * @param len length to read
 * @param pData pointer of data buffer
 * @return HAL_StatusTypeDef 只有 HAL_OK or HAL_ERROR 兩種可能
 */
HAL_StatusTypeDef DRV_Read(const DRV2605L_t* const pDrv,
                           const DRV_Regs iaddr,
                           uint16_t len,
                           uint8_t* const pData) {
  if (pDrv == NULL)
    return HAL_ERROR;

  if (pDrv->hi2c == NULL)
    return HAL_ERROR;

  if (pData == NULL)
    return HAL_ERROR;

  if (iaddr > LRA_Period)
    return HAL_ERROR;

  // 共有 0x23 (DRV_Total_Reg_Num) 個 registers
  if (len > (DRV_Total_Reg_Num - iaddr))
    len = DRV_Total_Reg_Num - iaddr;

  return DRV_UnsafeRead(pDrv, iaddr, len, pData);
}
