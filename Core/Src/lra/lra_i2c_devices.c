/*
 * lra_i2c.c
 *
 *  Created on: Feb 9, 2023
 *      Author: Dennis
 */

/* includes */

#include "lra/lra_i2c_devices.h"

#ifdef LRA_SYSTEM_INFO 
#include "lra/lra_usb.h"
#endif

/* public functions */

/**
 * @brief Try to init all TCA_DRV pairs, note that this function will reset
 * DRV2605L. Note that this function will be blocked by DRV2605L_SoftReset()
 *
 * @param pDevs
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef LRA_I2C_Devs_Init(const LRA_I2C_Devs_t* const pDevs) {
  if (pDevs == NULL)
    return HAL_ERROR;

  for (uint8_t i = 0; i < pDevs->pair_count; ++i) {

    TCA_DRV_Pair_t* pPair = pDevs->pDevPair[i];

    // switch to correct channel first
    HAL_StatusTypeDef ret = TCA_DRV_Pair_SwitchCH(pPair);
    if (ret != HAL_OK)
      return ret;

    /* set drv2605l parameters by calling DRV2605L lib functions */

    // id validate
    int8_t device_verify = DRV2605L_ID_Validate(pPair->pDrv);

    if ( device_verify != HAL_OK)
      continue;

    #ifdef LRA_SYSTEM_INFO
    LRA_USB_Print("LRA DevPair[%d] identifies as DRV2605L, starts to init process\r\n", i);
    #endif

    ret = DRV2605L_WriteReg(pPair->pDrv, DRV2605L_Mode, 0x07);

    // reset device
    ret = DRV2605L_SoftReset(pPair->pDrv);
    if (ret != HAL_OK)
      return ret;

    // set to PWM mode
    ret = DRV2605L_SetMode_PWM(pPair->pDrv);
    if (ret != HAL_OK)
      return ret;

    // custom settings
    ret = DRV2605L_Custom_Config(pPair->pDrv);
    if (ret != HAL_OK)
      return ret;

    // check everything is ok. Depending on LRA_main.h
    #ifdef LRA_DEBUG 
    uint8_t test[DRV2605L_Total_Reg_Num] = {0};
    ret = DRV2605L_Read_All(pPair->pDrv, test);
    if (ret != HAL_OK)
      return ret;
    #endif

    #ifdef LRA_SYSTEM_INFO
    #endif
  }

  return HAL_OK;
}

/**
 * @brief Change Channel function, 需要在 call DRV2605L 原生的函式前先行呼叫
 *
 * @param pDevPair
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef TCA_DRV_Pair_SwitchCH(const TCA_DRV_Pair_t* const pDevPair) {
  if (pDevPair == NULL || pDevPair->pTca == NULL)
    return HAL_ERROR;

  // checking channel is correct
  if (pDevPair->pTca->ch != pDevPair->located_ch)
    return TCA9546_Modify_CH(pDevPair->pTca, pDevPair->located_ch);

  return HAL_OK;
}

/* IO functions */

/**
 * @brief 寫入安裝在 TCA9546 上的 DRV2605L
 *
 * @details pDevPair->pDrv 和 pData 將會在 DRV2605L_Write 裡面檢查
 * @param pDevPair
 * @param iaddr
 * @param len
 * @param pData
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef TCA_DRV_Pair_Write(const TCA_DRV_Pair_t* const pDevPair,
                                     const DRV_Regs iaddr,
                                     const uint16_t len,
                                     uint8_t* const pData) {
  HAL_StatusTypeDef ret = TCA_DRV_Pair_SwitchCH(pDevPair);
  if (ret != HAL_OK)
    return ret;

  // write to target
  return DRV2605L_Write(pDevPair->pDrv, iaddr, len, pData);
}

/**
 * @brief
 *
 * @param pDevPair
 * @param iaddr
 * @param len
 * @param pData
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef TCA_DRV_Pair_Read(const TCA_DRV_Pair_t* const pDevPair,
                                    const DRV_Regs iaddr,
                                    const uint16_t len,
                                    uint8_t* const pData) {
  HAL_StatusTypeDef ret = TCA_DRV_Pair_SwitchCH(pDevPair);
  if (ret != HAL_OK)
    return ret;

  // read from target
  return DRV2605L_Read(pDevPair->pDrv, iaddr, len, pData);
}

/**
 * @brief
 *
 * @param pDevPair
 * @param iaddr
 * @param data
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef TCA_DRV_Pair_WriteReg(const TCA_DRV_Pair_t* const pDevPair,
                                        const DRV_Regs iaddr,
                                        const uint8_t data) {
  HAL_StatusTypeDef ret = TCA_DRV_Pair_SwitchCH(pDevPair);
  if (ret != HAL_OK)
    return ret;

  return DRV2605L_WriteReg(pDevPair->pDrv, iaddr, data);
}

/**
 * @brief
 *
 * @param pDevPair
 * @param iaddr
 * @param data
 * @return HAL_StatusTypeDef
 */
HAL_StatusTypeDef TCA_DRV_Pair_ReadReg(const TCA_DRV_Pair_t* const pDevPair,
                                       const DRV_Regs iaddr,
                                       uint8_t* const data) {
  HAL_StatusTypeDef ret = TCA_DRV_Pair_SwitchCH(pDevPair);
  if (ret != HAL_OK)
    return ret;

  return DRV2605L_ReadReg(pDevPair->pDrv, iaddr, data);
}
