#pragma once

/**
 * ---------------------------- Include ----------------------------
 */
#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"

/**
 * ---------------------------- Marco ----------------------------
 */
// The bit number of the parameters which will be sent to motor
#define SEND_POSITION_BIT 16
#define SEND_VELOCITY_BIT 14
#define SEND_KP_BIT 10
#define SEND_KD_BIT 8
#define SEND_TORQUE_BIT 16

// The bit number of the parameters which will be received by computer from motor
#define RECEIVE_CURRENT_POSITION_BIT 20
#define RECEIVE_CURRENT_VELOCITY_BIT 20
#define RECEIVE_CURRENT_TORQUE_BIT 16
#define RECEIVE_TEMPERATURE_FLAG_BIT 1
#define RECEIVE_CURRENT_TEMPERATURE_BIT 7

// The bit number of the parameters which will be received by computer from motor with error
#define ERROR_CODE_BIT 16
#define ERROR_VOLTAGE_BIT 16
#define ERROR_CURRENT_BIT 16
#define ERROR_MOTOR_TEMP_BIT 8
#define ERROR_MOSFET_TEMP_BIT 8

// Command
#define DISABLE_MOTOR 1
#define ENABLE_MOTOR 2
#define CALIBRATE_START 3
#define CONTROL_MOTOR 4
#define RESET_MOTOR 5
#define SET_HOME 6
#define SET_GEAR 7
#define SET_ID 8
#define SET_CAN_TIMEOUT 9
#define SET_BANDWIDTH 10
#define SET_LIMIT_CURRENT 11
#define SET_UNDER_VOLTAGE 12
#define SET_OVER_VOLTAGE 13
#define SET_MOTOR_TEMPERATURE 14
#define SET_DRIVE_TEMPERATURE 15
#define SAVE_CONFIG 16
#define ERROR_RESET 17
#define WRITE_APP_BACK_START 18
#define WRITE_APP_BACK 19
#define CHECK_APP_BACK 20
#define DFU_START 21
#define GET_FW_VERSION 22
#define GET_STATUS_WORD 23
#define GET_CONFIG 24
#define CALIB_REPORT 31

// Send DLC
#define SEND_DLC_DISABLE_MOTOR 0
#define SEND_DLC_ENABLE_MOTOR 0
#define SEND_DLC_CALIBRATE_START 0
#define SEND_DLC_CONTROL_MOTOR 8
#define SEND_DLC_RESET_MOTOR 0
#define SEND_DLC_SET_HOME 0
#define SEND_DLC_SET_GEAR 2
#define SEND_DLC_SET_ID 1
#define SEND_DLC_SET_CAN_TIMEOUT 1
#define SEND_DLC_SET_BANDWIDTH 2
#define SEND_DLC_SET_LIMIT_CURRENT 2
#define SEND_DLC_SET_UNDER_VOLTAGE 2
#define SEND_DLC_SET_OVER_VOLTAGE 2
#define SEND_DLC_SET_MOTOR_TEMPERATURE 2
#define SEND_DLC_SET_DRIVE_TEMPERATURE 2
#define SEND_DLC_SAVE_CONFIG 0
#define SEND_DLC_ERROR_RESET 0
#define SEND_DLC_WRITE_APP_BACK_START 0
#define SEND_DLC_WRITE_APP_BACK 8
#define SEND_DLC_CHECK_APP_BACK 8
#define SEND_DLC_DFU_START 0
#define SEND_DLC_GET_FW_VERSION 0
#define SEND_DLC_GET_STATUS_WORD 0
#define SEND_DLC_GET_CONFIG 0
#define SEND_DLC_CALIB_REPORT 8

// Receive DLC
#define RECEIVE_DLC_DISABLE_MOTOR 1
#define RECEIVE_DLC_ENABLE_MOTOR 1
#define RECEIVE_DLC_CALIBRATE_START 1
#define RECEIVE_DLC_CONTROL_MOTOR 8
#define RECEIVE_DLC_RESET_MOTOR 1
#define RECEIVE_DLC_SET_HOME 1
#define RECEIVE_DLC_SET_GEAR 2
#define RECEIVE_DLC_SET_ID 1
#define RECEIVE_DLC_SET_CAN_TIMEOUT 1
#define RECEIVE_DLC_SET_BANDWIDTH 1
#define RECEIVE_DLC_SET_LIMIT_CURRENT 1
#define RECEIVE_DLC_SET_UNDER_VOLTAGE 1
#define RECEIVE_DLC_SET_OVER_VOLTAGE 1
#define RECEIVE_DLC_SET_MOTOR_TEMPERATURE 1
#define RECEIVE_DLC_SET_DRIVE_TEMPERATURE 1
#define RECEIVE_DLC_SAVE_CONFIG 1
#define RECEIVE_DLC_ERROR_RESET 1
#define RECEIVE_DLC_WRITE_APP_BACK_START 1
#define RECEIVE_DLC_WRITE_APP_BACK 1
#define RECEIVE_DLC_CHECK_APP_BACK 2
#define RECEIVE_DLC_DFU_START 1
#define RECEIVE_DLC_GET_FW_VERSION 2
#define RECEIVE_DLC_GET_STATUE_WORD 5
#define RECEIVE_DLC_GET_CONFIG 8
#define RECEIVE_DLC_CALIB_REPORT 8

// Used in calculate Can ID
#define CAN_ID_SHIFT_BITS 5

// Program status
#define CAN_NORMAL 1
#define CAN_ERROR 0

/**
 * ---------------------------- Typedef ----------------------------
 */
// Sent data
typedef struct {
    uint32_t Position;
    uint32_t Velocity;
    uint32_t Kp;
    uint32_t Kd;
    uint32_t Torque;
} SendDataInformation;

// Received data
typedef struct {
    uint32_t CurrentPosition;
    uint32_t CurrentVelocity;
    uint32_t CurrentTorque;
    uint32_t TemperatureFlag;
    uint32_t CurrentTemperature;
} ReceiveDataInformation;

// The struct to store two type of sent can data
typedef struct {
    uint8_t data[8];               // Store real data according to transmit protocol
    SendDataInformation SendData;  // Real data which are needed to be sent in can bus
} SendCanDataInformaiton;

// The struct to store two type of received can data
typedef struct {
    uint8_t data[8];                     // Store real data according to transmit protocol
    ReceiveDataInformation ReceiveData;  // Real data which are received in can bus
} ReceiveCanDataInformation;

// Do more typedef to package more
typedef CAN_TxHeaderTypeDef CanTxStruct;
typedef CAN_RxHeaderTypeDef CanRxStruct;
typedef CAN_HandleTypeDef CanHandle;

// Can frame
typedef struct {
    uint16_t ID;                               // The id in can bus
    uint8_t DLC;                               // The bit number of data
    SendCanDataInformaiton SendCanData;        // Data sent to can bus according to command of controlling motor
    ReceiveCanDataInformation ReceiveCanData;  // Data received from can bus by command of controlling motor
    uint8_t SetConfigSend[2];                  // Data sent to can bus according to command of setting configuration (can timeout or band width)
    uint8_t GetConfigReceive[8];               // Data received from can bus by command of getting configuration
    uint8_t ReceiveStatusWord[2];              // Data received from can bus by command of getting status word
    uint8_t NormalCommandSend[8];              // Data sent to can bus according to normal command
    uint8_t NormalCommandStatus[1];            // Data received from can bus by normal command to tell us whether there is error
} CanFrame;

/**
 * ---------------------------- Variable ----------------------------
 */
extern CanFrame Can;                  // Can frame variable
extern uint32_t TxMailBox;            // Be used to store transmit mail box number
extern CanRxStruct CanRxInformation;  // Store received data in can bus

/**
 * ---------------------------- Function ----------------------------
 */
uint8_t StartCan (CanHandle* hcan);
uint8_t StopCan (CanHandle* hcan);
uint32_t RealDataToCanData (float RealData, float MinRealData, float MaxRealData, uint32_t DataBitNum);
float CanDataToRealData (uint32_t CanData, float MinRealData, float MaxRealData, uint32_t DataBitNum);
uint8_t Can_Send (CanFrame* Can, CanHandle* hcan);
uint8_t Can_Receive (CanFrame* Can, CanHandle* hcan);
