#pragma once

/**
 * ---------------------------- Include ----------------------------
 */
#include "DeepJ60_can.h"

/**
 * ---------------------------- Marco ----------------------------
 */
// Motor parameters limit
#define POSITION_MAX (40.0f)             // unit: rad
#define POSITION_MIN (-40.0f)            // unit: rad
#define VELOCITY_MAX (40.0f)             // unit: rad/s
#define VELOCITY_MIN (-40.0f)            // unit: rad/s
#define KP_MAX (1023.0f)
#define KP_MIN (0.0f)
#define KD_MAX (51.0f)
#define KD_MIN (0.0f)
#define TORQUE_MAX (40.0f)               // unit: N*m
#define TORQUE_MIN (-40.0f)              // unit: N*m
#define CURRENT_MAX (40.0f)              // unit: A
#define CURRENT_MIN (0.0f)               // unit: A
#define MOTOR_TEMPERATURE_MAX (200.0f)   // unit: Celsius
#define MOTOR_TEMPERATURE_MIN (-20.0f)   // unit: Celsius
#define MOSFET_TEMPERATURE_MAX (200.0f)  // unit: Celsius
#define MOSFET_TEMPERATURE_MIN (-20.0f)  // unit: Celsius
#define CAN_TIMEOUT_MAX 255              // unit: ms
#define CAN_TIMEOUT_MIN 0                // unit: ms
#define CURRENT_BAND_WIDTH_MAX 65535     // unit: Hz
#define CURRENT_BAND_WIDTH_MIN 0         // unit: Hz

// Function Status
#define NORMAL 1
#define ERROR 0

// The number of motor
#define MOTOR_NUMBER 8

/**
 * ---------------------------- Typedef ----------------------------
 */
// The information of motor id and motion parameters
typedef struct {
    uint8_t ID;
    float Position;
    float Velocity;
    float Kp;
    float Kd;
    float Torque;
} MotorDataInformation;

// The command to motor
typedef struct {
    uint8_t ID;
    MotorDataInformation MotorData;
} MotorCommandInformation;

// The motor data which is received from can bus
typedef struct {
    float CurrentPosition;
    float CurrentVelocity;
    float CurrentTorque;
    uint8_t TemperatureFlag;
    float CurrentTemperature;
} ReceiveMotorDataInformation;

// Motor configuration
typedef struct {
    uint8_t CanTimeout;
    uint16_t CurrentBandWidth;
    uint16_t CurrentLimit;
} MotorConfigInformation;

// The data returned by "Get Config" command
typedef struct {
    uint16_t InternalParameter;
    uint8_t MotorID;
    uint8_t CanTimeout;
    uint16_t CurrentBandWidth;
    uint16_t CurrentLimit;
} GetConfigReceiveDataInformation;

// Motor information
typedef struct {
    uint8_t ID;
    MotorCommandInformation MotorCommand;
    ReceiveMotorDataInformation ReceiveMotorData;
    MotorConfigInformation MotorConfig;
    GetConfigReceiveDataInformation GetConfigReceiveData;
    uint16_t MotorStatusWord;
    uint8_t MotorCommandStatus;
    uint8_t MotorMode;
    uint8_t CanNum;
} MotorInformation;

/**
 * ---------------------------- Variable ----------------------------
 */
// The temperature flag that may be received
enum Temperature_Flag {
    MosfetTemperatureFlag = 0,
    MotorTemperatureFlag
};

// Motor control mode
enum MotorMode {
    PositionMode = 0,
    VelocityMode,
    TorqueMode,
    KdMode,
    ZeroTorqueMode,
    PositionTorqueMode,
    VelocityTorqueMode,
    PositionVelocityTorqueMode
};

// Motor error type
enum MotorError {
    NoError = 0,
    OverVoltageError = 0x01,
    UnderVoltageError = (0x01 << 1),
    OverCurrentError = (0x01 << 2),
    MotorOverTemperatureError = (0x01 << 3),
    DriverOverTemperatureError = (0x01 << 4),
    CanTimeoutError = (0x01 << 5),
    TemperatureSensorDisconnectedError = (0x01 << 6)
};

extern MotorInformation J60Motor_CAN1[MOTOR_NUMBER];  // The information struct of J60-Motor in can1 bus
extern MotorInformation J60Motor_CAN2[MOTOR_NUMBER];  // The information struct of J60-Motor in can2 bus

/**
 * ---------------------------- Function ----------------------------
 */
// Start or stop command
uint8_t StartJ60MotorCommand (MotorInformation* Motor);
uint8_t StopJ60MotorCommand (MotorInformation* Motor);

// Set parameters
uint8_t SetMotionParameters (MotorInformation* Motor, float Position, float Velocity, float Kp, float Kd, float Torque);
uint8_t SetMotionCommand (MotorInformation* Motor, uint8_t CommandID, float Position, float Velocity, float Kp, float Kd, float Torque);
void SetNormalCommand (MotorInformation* Motor, uint8_t CommandID);

// Data convertion
void J60MotorMotionDataToSendCanData (MotorDataInformation* MotorData, SendCanDataInformaiton* SendCanData);
void ReceiveCanDataToJ60MotorMotionData (ReceiveMotorDataInformation* ReceiveMotorData, ReceiveCanDataInformation* ReceiveCanData);
void J60MotorConfigToSendCanData (MotorConfigInformation* MotorConfig, uint8_t* SetConfigSend, uint8_t CommandID);
void ReceiveCanDataToJ60MotorConfig (GetConfigReceiveDataInformation* GetConfigReceiveData, uint8_t* GetConfigReceive);
void ReceiveCommandStatusToJ60MotorCommandStatus (uint8_t* MotorCommandStatus, uint8_t* ReceiveNormalCommandStatus);
void ReceiveStatusWordToJ60MotorStatusWord (uint16_t* MotorStatusWord, uint8_t* ReceiveStatusWord);
void J60MotorDataToSendCanData (MotorInformation* Motor, CanFrame* Can);
void ReceiveCanDataToJ60MotorData (MotorInformation* Motor, CanFrame* Can);
uint16_t GetJ60MotorCanID (MotorCommandInformation* MotorCommand);

// Give motor command
uint8_t DisableJ60Motor (MotorInformation* Motor);
uint8_t EnableJ60Motor (MotorInformation* Motor, uint8_t MotorID, uint8_t CanNum);
uint8_t GetJ60MotorConfig (MotorInformation* Motor);
uint8_t ConfigJ60MotorCanTimeout (MotorInformation* Motor, int CanTimeout);
uint8_t ConfigJ60MotorCurrentBandWidth (MotorInformation* Motor, int CurrentBandWidth);
uint8_t SaveJ60MotorConfig (MotorInformation* Motor);
uint8_t RunJ60MotorPositionMode (MotorInformation* Motor, float Position, float Kp, float Kd);
uint8_t RunJ60MotorVelocityMode (MotorInformation* Motor, float Velocity, float Kd);
uint8_t RunJ60MotorTorqueMode (MotorInformation* Motor, float Torque);
uint8_t RunJ60MotorKdMode (MotorInformation* Motor, float Kd);
uint8_t RunJ60MotorZeroTorqueMode (MotorInformation* Motor);
uint8_t RunJ60MotorPositionTorqueMode (MotorInformation* Motor, float Position, float Torque, float Kp, float Kd);
uint8_t RunJ60MotorVelocityTorqueMode (MotorInformation* Motor, float Velocity, float Torque, float Kd);
uint8_t RunJ60MotorPositonVelocityTorqueMode (MotorInformation* Motor, float Position, float Velocity, float Torque, float Kp, float Kd);
uint8_t RunJ60Motor (MotorInformation* Motor, float Position, float Velocity, float Torque, float Kp, float Kd, enum MotorMode Mode);
uint8_t GetJ60MotorStatusWord (MotorInformation* Motor);
uint8_t J60MotorErrorReset (MotorInformation* Motor);

// Analyse received data
void AnalyseJ60MotorReceiveData (MotorInformation* Motor);
