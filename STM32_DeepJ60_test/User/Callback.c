#include "DeepJ60_can.h"
#include "DeepJ60_Motor.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_can.h"

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN1 || hcan->Instance == CAN2) {
        Can_Receive(&Can, hcan);
        AnalyseJ60MotorReceiveData();
    }
}
