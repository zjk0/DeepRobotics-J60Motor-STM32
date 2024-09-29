/**
 * ---------------------------- Include ----------------------------
 */
#include "DeepJ60_can.h"

/**
 * ---------------------------- Variable ----------------------------
 */
CanFrame Can;
uint32_t TxMailBox;
CAN_RxHeaderTypeDef CanRxInformation;

/**
 * ---------------------------- Function ----------------------------
 */
/**
 * @brief Start can transmission
 * 
 * @param hcan: The pointer to the handle of can
 * 
 * @return uint8_t: CAN_ERROR: Error happened while starting can transmission
 *                  CAN_NORMAL: It is normal while starting can transmission
 */
uint8_t StartCan (CanHandle* hcan) {
    if (HAL_CAN_Start(hcan) != HAL_OK) {
        return CAN_ERROR;
    }
    return CAN_NORMAL;
}

/**
 * @brief Stop can transmission
 * 
 * @param hcan: The pointer to the handle of can
 * 
 * @return uint8_t: CAN_ERROR: Error happened while stopping can transmission
 *                  CAN_NORMAL: It is normal while stopping can transmission
 */
uint8_t StopCan (CanHandle* hcan) {
    if (HAL_CAN_Stop(hcan) != HAL_OK) {
        return CAN_ERROR;
    }
    return CAN_NORMAL;
}

/**
 * @brief Convert the real data with unit to the data in can bus
 * 
 * @param RealData: The real data that need to be converted
 * @param MinRealData: The min value of real data
 * @param MaxRealData: The max value of real data
 * @param DataBitNum: The bit number of real data
 * 
 * @return uint32_t: The data in can bus after converting
 */
uint32_t RealDataToCanData (float RealData, float MinRealData, float MaxRealData, uint32_t DataBitNum) {
    return (uint32_t)(((1 << DataBitNum) - 1) * (RealData - MinRealData) / (MaxRealData - MinRealData));
}

/**
 * @brief Convert the data in can bus to the real data with unit
 * 
 * @param CanData: The data in can bus that need to be converted
 * @param MinRealData: The min value of real data
 * @param MaxRealData: The max value of real data
 * @param DataBitNum: The bit number of real data
 * 
 * @return float: The real data with unit after converting
 */
float CanDataToRealData (uint32_t CanData, float MinRealData, float MaxRealData, uint32_t DataBitNum) {
    return (MinRealData + (MaxRealData - MinRealData) * CanData / ((1 << DataBitNum) - 1));
}

/**
 * @brief Send data to can bus
 * 
 * @param Can: The pointer to the struct of can frame
 * @param hcan: The pointer to the handle of can
 * 
 * @return uint8_t: CAN_ERROR: Error happened while sending data to can bus
 *                  CAN_NORMAL: It is normal while sending data to can bus
 */
uint8_t Can_Send (CanFrame* Can, CanHandle* hcan) {
    CanTxStruct CanTxInformation;

    CanTxInformation.StdId = Can->ID;
    CanTxInformation.IDE = CAN_ID_STD;
    CanTxInformation.RTR = CAN_RTR_DATA;
    CanTxInformation.DLC = Can->DLC;
    CanTxInformation.TransmitGlobalTime = DISABLE;

    if (Can->ID >> CAN_ID_SHIFT_BITS == CONTROL_MOTOR) {
        if (HAL_CAN_AddTxMessage(hcan, &CanTxInformation, Can->SendCanData.data, &TxMailBox) != HAL_OK) {
            return CAN_ERROR;
        }
    }
    else if (Can->ID >> CAN_ID_SHIFT_BITS == SET_CAN_TIMEOUT || Can->ID >>CAN_ID_SHIFT_BITS == SET_BANDWIDTH) {
        if (HAL_CAN_AddTxMessage(hcan, &CanTxInformation, Can->SetConfigSend, &TxMailBox) != HAL_OK) {
            return CAN_ERROR;
        }
    }
    else {
        if (HAL_CAN_AddTxMessage(hcan, &CanTxInformation, Can->NormalCommandSend, &TxMailBox) != HAL_OK) {
            return CAN_ERROR;
        }
    }
    return CAN_NORMAL;
}

/**
 * @brief Receive data from can bus
 * 
 * @param Can: The pointer to the struct of can frame
 * @param hcan: The pointer to the handle of can
 * 
 * @return uint8_t: CAN_ERROR: Error happened while receiving data from can bus
 *                  CAN_NORMAL: It is normal while receiving data from can bus
 */
uint8_t Can_Receive (CanFrame* Can, CanHandle* hcan) {
    if (Can->ID >> CAN_ID_SHIFT_BITS == CONTROL_MOTOR) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxInformation, Can->ReceiveCanData.data) != HAL_OK) {
            return CAN_ERROR;
        }
    }
    else if (Can->ID >> CAN_ID_SHIFT_BITS == GET_CONFIG) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxInformation, Can->GetConfigReceive) != HAL_OK) {
            return CAN_ERROR;
        }
    }
    else if (Can->ID >> CAN_ID_SHIFT_BITS == GET_STATUS_WORD) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxInformation, Can->ReceiveStatusWord) != HAL_OK) {
            return CAN_ERROR;
        }
    }
    else {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CanRxInformation, Can->NormalCommandStatus) != HAL_OK) {
            return CAN_ERROR;
        }
    }
    return CAN_NORMAL;
}
