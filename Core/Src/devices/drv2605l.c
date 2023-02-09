/*
 * drv2605l.c
 *
 *  Created on: Feb 9, 2023
 *      Author: Dennis
 */

/* includes */

#include "devices/drv2605l.h"

/* exported variables */
const uint8_t drv2605l_addr_default = 0x5A;

/* static function declaration */

static HAL_StatusTypeDef DRV_UnsafeRead(const DRV2605L_t* const pDrv,
                                        const DRV_Regs iaddr,
                                        const uint16_t len,
                                        uint8_t* const pData);

static HAL_StatusTypeDef DRV_UnsafeWrite(const DRV2605L_t* const pDrv,
                                         const DRV_Regs iaddr,
                                         const uint16_t len,
                                         uint8_t* const pData);

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

  uint8_t val = 0x01 << 7;
  return DRV_UnsafeWrite(pDrv, Mode, 1, &val);
}

/**
 * @brief Enable device to execute cmd
 *
 * @param drv
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_Go(const DRV2605L_t* const pDrv) {
  if (pDrv == NULL || pDrv->hi2c == NULL)
    return HAL_ERROR;

  uint8_t val = 0x01;

  return DRV_UnsafeWrite(pDrv, Go, 1, &val);
}

/**
 * @brief Disable device
 *
 * @param pDrv
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_Stop(const DRV2605L_t* const pDrv) {
  if (pDrv == NULL || pDrv->hi2c == NULL)
    return HAL_ERROR;

  uint8_t val = 0x00;

  return DRV_UnsafeWrite(pDrv, Go, 1, &val);
}

/**
 * @brief Set DRV2605L to PWM control mode
 *
 * @param pDrv
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef DRV_Set_Mode_PWM(const DRV2605L_t* const pDrv) {}

/**
 * @brief 
 * 
 * @param pDrv 
 * @param pData 
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef DRV_Read_All(const DRV2605L_t* const pDrv, uint8_t* pData) {
  if (pDrv == NULL || pDrv->hi2c == NULL)
    return HAL_ERROR;

  return DRV_UnsafeRead(pDrv, Status, DRV_Total_Reg_Num, pData);
}

/* IO functions */

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
