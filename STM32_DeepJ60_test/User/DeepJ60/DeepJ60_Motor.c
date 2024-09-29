/**
 * ---------------------------- Include ----------------------------
 */
#include "DeepJ60_Motor.h"
#include "DeepJ60_can.h"

/**
 * ---------------------------- Variable ----------------------------
 */
MotorInformation J60Motor[MOTOR_NUMBER];  // The information struct of J60-Motor

/**
 * ---------------------------- Function ----------------------------
 */
/**
 * @brief Start to transmit command to motor by can bus
 * 
 * @param none
 * 
 * @return none
 */
uint8_t StartJ60MotorCommand (void) {
    if (StartCan(&hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Stop transmitting command to motor
 * 
 * @param none
 * 
 * @return none
 */
uint8_t StopJ60MotorCommand (void) {
    if (StopCan(&hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Set motion parameters for motor J60
 * 
 * @param Motor: The pointer to the struct to store motor information
 * @param Position: The position of motor (rad)
 * @param Velocity: The velocity of motor (rad/s)
 * @param Kp: The stiffness coefficient of motor (no unit)
 * @param Kd: The damping coefficient of motor (no unit)
 * @param Torque: The torque of motor (N*m)
 * 
 * @return uint8_t: ERROR: Error happened while setting motion parameters
 *                  NORMAL: Successfully set motion parameters
 */
uint8_t SetMotionParameters (MotorInformation* Motor, float Position, float Velocity, float Kp, float Kd, float Torque) {
    if (Position > POSITION_MAX || Position < POSITION_MIN) {
        return ERROR;
    }
    else if (Velocity > VELOCITY_MAX || Velocity < VELOCITY_MIN) {
        return ERROR;
    }
    else if (Kp > KP_MAX || Kp < KP_MIN) {
        return ERROR;
    }
    else if (Kd > KD_MAX || Kd < KD_MIN) {
        return ERROR;
    }
    else if (Torque > TORQUE_MAX || Torque < TORQUE_MIN) {
        return ERROR;
    }

    Motor->MotorCommand.MotorData.Position = Position;
    Motor->MotorCommand.MotorData.Velocity = Velocity;
    Motor->MotorCommand.MotorData.Kp = Kp;
    Motor->MotorCommand.MotorData.Kd = Kd;
    Motor->MotorCommand.MotorData.Torque = Torque;

    return NORMAL;
}

/**
 * @brief Set motion command for motor J60, include motion parameters and command id
 * 
 * @param Motor: The pointer to the struct to store motor information
 * @param CommandID: The ID of command
 * @param Position: The position of motor (rad)
 * @param Velocity: The velocity of motor (rad/s)
 * @param Kp: The stiffness coefficient of motor (no unit)
 * @param Kd: The damping coefficient of motor (no unit)
 * @param Torque: The torque of motor (N*m)
 * 
 * @return uint8_t: ERROR: Error happened while setting motion command
 *                  NORMAL: Successfully set motion command
 */
uint8_t SetMotionCommand (MotorInformation* Motor, uint8_t CommandID, float Position, float Velocity, float Kp, float Kd, float Torque) {
    Motor->MotorCommand.ID = CommandID;
    Motor->MotorCommand.MotorData.ID = Motor->ID;

    if (SetMotionParameters (Motor, Position, Velocity, Kp, Kd, Torque) == ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Set simple command for motor J60, which do not need to send many parameters to motor
 * 
 * @param Motor: The pointer to the struct to store motor information
 * @param CommandID: The ID of command
 * 
 * @return none
 */
void SetNormalCommand (MotorInformation* Motor, uint8_t CommandID) {
    Motor->MotorCommand.ID = CommandID;
    Motor->MotorCommand.MotorData.ID = Motor->ID;
}

/**
 * @brief Convert the motion parameters of motor to the data in can bus while using the command to control motor
 * 
 * @param MotorData: The pointer to the struct which stores the motion parameters
 * @param SendCanData: The pointer to the struct which stores the related data in can bus
 * 
 * @return none
 */
void J60MotorMotionDataToSendCanData (MotorDataInformation* MotorData, SendCanDataInformaiton* SendCanData) {
    SendCanData->SendData.Position = RealDataToCanData(MotorData->Position, 
                                                       POSITION_MIN, 
                                                       POSITION_MAX, 
                                                       SEND_POSITION_BIT);
    SendCanData->SendData.Velocity = RealDataToCanData(MotorData->Velocity, 
                                                       VELOCITY_MIN, 
                                                       VELOCITY_MAX, 
                                                       SEND_VELOCITY_BIT);
    SendCanData->SendData.Kp = RealDataToCanData(MotorData->Kp, 
                                                 KP_MIN, 
                                                 KP_MAX, 
                                                 SEND_KP_BIT);
    SendCanData->SendData.Kd = RealDataToCanData(MotorData->Kd, 
                                                 KD_MIN, 
                                                 KD_MAX, 
                                                 SEND_KD_BIT);
    SendCanData->SendData.Torque = RealDataToCanData(MotorData->Torque, 
                                                     TORQUE_MIN, 
                                                     TORQUE_MAX, 
                                                     SEND_TORQUE_BIT);

    SendCanData->data[0] = SendCanData->SendData.Position & 0xFF;
    SendCanData->data[1] = (SendCanData->SendData.Position >> 8) & 0xFF;
    SendCanData->data[2] = SendCanData->SendData.Velocity & 0xFF;
    SendCanData->data[3] = ((SendCanData->SendData.Velocity >> 8) & 0x3F) | ((SendCanData->SendData.Kp & 0x03) << 6);
    SendCanData->data[4] = (SendCanData->SendData.Kp >> 2) & 0xFF;
    SendCanData->data[5] = SendCanData->SendData.Kd & 0xFF;
    SendCanData->data[6] = SendCanData->SendData.Torque & 0xFF;
    SendCanData->data[7] = (SendCanData->SendData.Torque >> 8) & 0xFF;
}

/**
 * @brief Convert the data received from can bus to the motion data of motor while using command to control motor
 * 
 * @param ReceiveMotorData: The pointer to the struct which stores the motion parameters returned by motor
 * @param ReceiveCanData: The pointer to the struct which stores the data received from can bus while using command to control motor
 * 
 * @return none
 */
void ReceiveCanDataToJ60MotorMotionData (ReceiveMotorDataInformation* ReceiveMotorData, ReceiveCanDataInformation* ReceiveCanData) {
    ReceiveCanData->ReceiveData.CurrentPosition = ReceiveCanData->data[0] | (ReceiveCanData->data[1] << 8) | ((ReceiveCanData->data[2] & 0x0F) << 16);
    ReceiveCanData->ReceiveData.CurrentVelocity = ((ReceiveCanData->data[2] >> 4) & 0x0F) | ((ReceiveCanData->data[3] << 4) & 0xFF0) | ((ReceiveCanData->data[4] << 12) & 0xFF000);
    ReceiveCanData->ReceiveData.CurrentTorque = (ReceiveCanData->data[5] & 0xFF) | ((ReceiveCanData->data[6] << 8) & 0xFF00);
    ReceiveCanData->ReceiveData.TemperatureFlag = ReceiveCanData->data[7] & 0x01;
    ReceiveCanData->ReceiveData.CurrentTemperature = (ReceiveCanData->data[7] >> 1) & 0x7F;

    ReceiveMotorData->CurrentPosition = CanDataToRealData(ReceiveCanData->ReceiveData.CurrentPosition, 
                                                          POSITION_MIN, 
                                                          POSITION_MAX, 
                                                          RECEIVE_CURRENT_POSITION_BIT);
    ReceiveMotorData->CurrentVelocity = CanDataToRealData(ReceiveCanData->ReceiveData.CurrentVelocity, 
                                                          VELOCITY_MIN, 
                                                          VELOCITY_MAX, 
                                                          RECEIVE_CURRENT_VELOCITY_BIT);
    ReceiveMotorData->CurrentTorque = CanDataToRealData(ReceiveCanData->ReceiveData.CurrentTorque, 
                                                        TORQUE_MIN, 
                                                        TORQUE_MAX, 
                                                        RECEIVE_CURRENT_TORQUE_BIT);
    ReceiveMotorData->TemperatureFlag = ReceiveCanData->ReceiveData.TemperatureFlag;
    if (ReceiveMotorData->TemperatureFlag == MosfetTemperatureFlag) {
        ReceiveMotorData->CurrentTemperature = CanDataToRealData(ReceiveCanData->ReceiveData.CurrentTemperature, 
                                                                 MOSFET_TEMPERATURE_MIN, 
                                                                 MOSFET_TEMPERATURE_MAX, 
                                                                 RECEIVE_CURRENT_TEMPERATURE_BIT);
    }
    else if (ReceiveMotorData->TemperatureFlag == MotorTemperatureFlag) {
        ReceiveMotorData->CurrentTemperature = CanDataToRealData(ReceiveCanData->ReceiveData.CurrentTemperature, 
                                                                 MOTOR_TEMPERATURE_MIN, 
                                                                 MOTOR_TEMPERATURE_MAX, 
                                                                 RECEIVE_CURRENT_TEMPERATURE_BIT);
    }
}

/**
 * @brief Convert motor configuration to the data in can bus while using command to config motor
 * 
 * @param MotorConfig: The pointer to the struct which stores the motor configuration
 * @param SetConfigSend: The pointer to the array which stores the data sent to can bus while using command to config motor
 * @param CommandID: The ID of command
 * 
 * @return none
 */
void J60MotorConfigToSendCanData (MotorConfigInformation* MotorConfig, uint8_t* SetConfigSend, uint8_t CommandID) {
    if (CommandID == SET_BANDWIDTH) {
        SetConfigSend[0] = MotorConfig->CurrentBandWidth & 0xFF;
        SetConfigSend[1] = (MotorConfig->CurrentBandWidth >> 8) & 0xFF;
    }
    else if (CommandID == SET_CAN_TIMEOUT) {
        SetConfigSend[0] = MotorConfig->CanTimeout & 0xFF;
        SetConfigSend[1] = 0;
    }
}

/**
 * @brief Convert the data received from can bus to the motor configuration while using command to get motor configuration
 * 
 * @param GetConfigReceivedata: The pointer to the struct which stores the motor configuration while using command to get configuration
 * @param GetConfigReceive: The pointer to the array which stores the data returned by motor while using command to get configuration
 * 
 * @return none
 */
void ReceiveCanDataToJ60MotorConfig (GetConfigReceiveDataInformation* GetConfigReceiveData, uint8_t* GetConfigReceive) {
    GetConfigReceiveData->InternalParameter = (GetConfigReceive[0] & 0xFF) | ((GetConfigReceive[1] << 8) & 0xFF00);
    GetConfigReceiveData->MotorID = GetConfigReceive[2];
    GetConfigReceiveData->CanTimeout = GetConfigReceive[3];
    GetConfigReceiveData->CurrentBandWidth = (GetConfigReceive[4] & 0xFF) | ((GetConfigReceive[5] << 8) & 0xFF00);
    GetConfigReceiveData->CurrentLimit = (GetConfigReceive[6] & 0xFF) | ((GetConfigReceive[7] << 8) & 0xFF00);
}

/**
 * @brief Convert received command status from can bus to the command status of motor
 * 
 * @param MotorCommandStatus: The pointer to the area which stores the command status of motor
 * @param ReceiveNormalCommandStatus: The pointer to the area which stores the command status returned by motor while using the normal command
 * 
 * @return none
 */
void ReceiveCommandStatusToJ60MotorCommandStatus (uint8_t* MotorCommandStatus, uint8_t* ReceiveNormalCommandStatus) {
    *MotorCommandStatus = ReceiveNormalCommandStatus[0];
}

/**
 * @brief Convert received status word from can bus to the status word of motor
 * 
 * @param MotorStatusWord: The pointer to the area which stores the status word of motor
 * @param ReceiveStatusWord: The pointer to the area which stores the status word returned by motor while using the command to get status word
 * 
 * @return none
 */
void ReceiveStatusWordToJ60MotorStatusWord (uint16_t* MotorStatusWord, uint8_t* ReceiveStatusWord) {
    if (ReceiveStatusWord[0] == OverVoltageError) {
        *MotorStatusWord = OverVoltageError;
    }
    else if (ReceiveStatusWord[0] == UnderVoltageError) {
        *MotorStatusWord = UnderVoltageError;
    }
    else if (ReceiveStatusWord[0] == OverCurrentError) {
        *MotorStatusWord = OverCurrentError;
    }
    else if (ReceiveStatusWord[0] == MotorOverTemperatureError) {
        *MotorStatusWord = MotorOverTemperatureError;
    }
    else if (ReceiveStatusWord[0] == DriverOverTemperatureError) {
        *MotorStatusWord = DriverOverTemperatureError;
    }
    else if (ReceiveStatusWord[0] == CanTimeoutError) {
        *MotorStatusWord = CanTimeoutError;
    }
    else if (ReceiveStatusWord[0] == TemperatureSensorDisconnectedError) {
        *MotorStatusWord = TemperatureSensorDisconnectedError;
    }
    else {
        *MotorStatusWord = NoError;
    }
}

/**
 * @brief Convert the data of motor to the data in can bus
 * 
 * @param Motor: The pointer to the motor information struct
 * @param Can: The pointer to the can frame struct
 * 
 * @return none
 */
void J60MotorDataToSendCanData (MotorInformation* Motor, CanFrame* Can) {
    if (Motor->MotorCommand.ID == CONTROL_MOTOR) {
        J60MotorMotionDataToSendCanData(&Motor->MotorCommand.MotorData, &Can->SendCanData);
    }
    else if (Motor->MotorCommand.ID == SET_BANDWIDTH || Motor->MotorCommand.ID == SET_CAN_TIMEOUT) {
        J60MotorConfigToSendCanData(&Motor->MotorConfig, Can->SetConfigSend, Motor->MotorCommand.ID);
    }
}

/**
 * @brief Convert the data returned by motor to the real data of motor
 * 
 * @param Motor: The pointer to the motor information struct
 * @param Can: The pointer to the can frame struct
 * 
 * @return none
 */
void ReceiveCanDataToJ60MotorData (MotorInformation* Motor, CanFrame* Can) {
    if (Motor->MotorCommand.ID == CONTROL_MOTOR) {
        ReceiveCanDataToJ60MotorMotionData(&Motor->ReceiveMotorData, &Can->ReceiveCanData);
    }
    else if (Motor->MotorCommand.ID == GET_CONFIG) {
        ReceiveCanDataToJ60MotorConfig(&Motor->GetConfigReceiveData, Can->GetConfigReceive);
    }
    else if (Motor->MotorCommand.ID == GET_STATUS_WORD) {
        ReceiveStatusWordToJ60MotorStatusWord(&Motor->MotorStatusWord, Can->ReceiveStatusWord);
    }
    else {
        ReceiveCommandStatusToJ60MotorCommandStatus(&Motor->MotorCommandStatus, Can->NormalCommandStatus);
    }
}

/**
 * @brief Get the can id of motor, which is calculated by motor id and command id
 * 
 * @param MotorCommand: The pointer to the struct which stores the information of motor command
 * 
 * @return uint16_t: The can id of motor
 */
uint16_t GetJ60MotorCanID (MotorCommandInformation* MotorCommand) {
    return (MotorCommand->MotorData.ID) | ((MotorCommand->ID) << CAN_ID_SHIFT_BITS);
}

/**
 * @brief Disable motor
 * 
 * @param Motor: The pointer to the struct to store motor information
 * 
 * @return uint8_t ERROR: Error happened while disabling motor
 *                 NORMAL: Successfully disable motor
 */
uint8_t DisableJ60Motor (MotorInformation* Motor) {
    SetNormalCommand(Motor, DISABLE_MOTOR);

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_DISABLE_MOTOR;

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }

    StopJ60MotorCommand();
    return NORMAL;
}

/**
 * @brief Enable motor
 * 
 * @param Motor: The pointer to the struct to store motor information
 * @param MotorID: The ID of motor
 * 
 * @return uint8_t ERROR: Error happened while enabling motor
 *                 NORMAL: Successfully enable motor
 */
uint8_t EnableJ60Motor (MotorInformation* Motor, uint8_t MotorID) {
	
	
	
    Motor->ID = MotorID;
    SetNormalCommand(Motor, ENABLE_MOTOR);

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_ENABLE_MOTOR;

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }

    StartJ60MotorCommand();
    return NORMAL;
}

/**
 * @brief Get motor configuration
 * 
 * @param Motor: The pointer to the struct to store motor information
 * 
 * @return uint8_t ERROR: Error happened while getting motor configuration
 *                 NORMAL: Successfully get motor configuration
 */
uint8_t GetJ60MotorConfig (MotorInformation* Motor) {
    SetNormalCommand(Motor, GET_CONFIG);

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_GET_CONFIG;

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Config the can timeout of motor
 * 
 * @param Motor: The pointer to the struct to store motor information
 * @param CanTimeout: The can timeout of motor
 * 
 * @return uint8_t ERROR: Error happened while setting motor can timeout
 *                 NORMAL: Successfully set motor can timeout
 */
uint8_t ConfigJ60MotorCanTimeout (MotorInformation* Motor, int CanTimeout) {
    SetNormalCommand(Motor, SET_CAN_TIMEOUT);
    if (CanTimeout > CAN_TIMEOUT_MAX || CanTimeout < CAN_TIMEOUT_MIN) {
        return ERROR;
    }
    Motor->MotorConfig.CanTimeout = CanTimeout;

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_SET_CAN_TIMEOUT;
    J60MotorDataToSendCanData(Motor, &Can);

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Config the current band width
 * 
 * @param Motor: The pointer to the struct to store motor information
 * @param CurrentBandWidth: The current band width of motor
 * 
 * @return uint8_t ERROR: Error happened while setting the current band width of motor
 *                 NORMAL: Successfully set the current band width of motor
 */
uint8_t ConfigJ60MotorCurrentBandWidth (MotorInformation* Motor, int CurrentBandWidth) {
    SetNormalCommand(Motor, SET_BANDWIDTH);
    if (CurrentBandWidth > CURRENT_BAND_WIDTH_MAX || CurrentBandWidth < CURRENT_BAND_WIDTH_MIN) {
        return ERROR;
    }
    Motor->MotorConfig.CurrentBandWidth = CurrentBandWidth;

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_SET_BANDWIDTH;
    J60MotorDataToSendCanData(Motor, &Can);

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Save the configuration of motor
 * 
 * @param Motor: The pointer to the struct to store motor information
 * 
 * @return uint8_t ERROR: Error happened while saving the configuration of motor
 *                 NORMAL: Successfully save the configuration of motor
 */
uint8_t SaveJ60MotorConfig (MotorInformation* Motor) {
    SetNormalCommand(Motor, SAVE_CONFIG);

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_SAVE_CONFIG;

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Run motor in position mode
 *
 * @param Motor: The pointer to the struct to store motor information
 * @param Position: The position that motor need to be in (unit: rad)
 * @param Kp: The stiffness coefficient of motor (no unit)
 * @param Kd: The damping coefficient of motor (no unit)
 * 
 * @return uint8_t ERROR: Error happened while motor is in position mode
 *                 NORMAL: It is normal when motor is in position mode  
 */
uint8_t RunJ60MotorPositionMode (MotorInformation* Motor, float Position, float Kp, float Kd) {
    if (SetMotionCommand(Motor, CONTROL_MOTOR, Position, 0, Kp, Kd, 0) == ERROR) {
        return ERROR;
    }

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_CONTROL_MOTOR;
    J60MotorDataToSendCanData(Motor, &Can);

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Run motor in velocity mode
 *
 * @param Motor: The pointer to the struct to store motor information
 * @param Velocity: The velocity of motor (unit: rad/s)
 * @param Kd: The damping coefficient of motor (no unit)
 * 
 * @return uint8_t ERROR: Error happened while motor is in velocity mode
 *                 NORMAL: It is normal when motor is in velocity mode  
 */
uint8_t RunJ60MotorVelocityMode (MotorInformation* Motor, float Velocity, float Kd) {
    if (SetMotionCommand(Motor, CONTROL_MOTOR, 0, Velocity, 0, Kd, 0) == ERROR) {
        return ERROR;
    }

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_CONTROL_MOTOR;
    J60MotorDataToSendCanData(Motor, &Can);

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Run motor in torque mode
 *
 * @param Motor: The pointer to the struct to store motor information
 * @param Torque: The torque of motor (unit: N*m)
 * 
 * @return uint8_t ERROR: Error happened while motor is in torque mode
 *                 NORMAL: It is normal when motor is in torque mode  
 */
uint8_t RunJ60MotorTorqueMode (MotorInformation* Motor, float Torque) {
    if (SetMotionCommand(Motor, CONTROL_MOTOR, 0, 0, 0, 0, Torque) == ERROR) {
        return ERROR;
    }

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_CONTROL_MOTOR;
    J60MotorDataToSendCanData(Motor, &Can);

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Run motor in Kd mode
 *
 * @param Motor: The pointer to the struct to store motor information
 * @param Kd: The damping coefficient of motor (no unit)
 * 
 * @return uint8_t ERROR: Error happened while motor is in Kd mode
 *                 NORMAL: It is normal when motor is in Kd mode  
 */
uint8_t RunJ60MotorKdMode (MotorInformation* Motor, float Kd) {
    if (SetMotionCommand(Motor, CONTROL_MOTOR, 0, 0, 0, Kd, 0) == ERROR) {
        return ERROR;
    }

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_CONTROL_MOTOR;
    J60MotorDataToSendCanData(Motor, &Can);

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Run motor in zero torque mode
 *
 * @param Motor: The pointer to the struct to store motor information
 * 
 * @return uint8_t ERROR: Error happened while motor is in zero torque mode
 *                 NORMAL: It is normal when motor is in zero torque mode  
 */
uint8_t RunJ60MotorZeroTorqueMode (MotorInformation* Motor) {
    if (SetMotionCommand(Motor, CONTROL_MOTOR, 0, 0, 0, 0, 0) == ERROR) {
        return ERROR;
    }

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_CONTROL_MOTOR;
    J60MotorDataToSendCanData(Motor, &Can);

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Run motor in position-torque mode
 *
 * @param Motor: The pointer to the struct to store motor information
 * @param Position: The position of motor (unit: rad)
 * @param Torque: The torque of motor (unit: N*m)
 * @param Kp: The stiffness coefficient of motor (no unit)
 * @param Kd: The damping coefficient of motor (no unit)
 * 
 * @return uint8_t ERROR: Error happened while motor is in position-torque mode
 *                 NORMAL: It is normal when motor is in position-torque mode  
 */
uint8_t RunJ60MotorPositionTorqueMode (MotorInformation* Motor, float Position, float Torque, float Kp, float Kd) {
    if (SetMotionCommand(Motor, CONTROL_MOTOR, Position, 0, Kp, Kd, Torque) == ERROR) {
        return ERROR;
    }

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_CONTROL_MOTOR;
    J60MotorDataToSendCanData(Motor, &Can);

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Run motor in velocity-torque mode
 *
 * @param Motor: The pointer to the struct to store motor information
 * @param Velocity: The velocity of motor (unit: rad/s)
 * @param Torque: The torque of motor (unit: N*m)
 * @param Kd: The damping coefficient of motor (no unit)
 * 
 * @return uint8_t ERROR: Error happened while motor is in velocity-torque mode
 *                 NORMAL: It is normal when motor is in velocity-torque mode  
 */
uint8_t RunJ60MotorVelocityTorqueMode (MotorInformation* Motor, float Velocity, float Torque, float Kd) {
    if (SetMotionCommand(Motor, CONTROL_MOTOR, 0, Velocity, 0, Kd, Torque) == ERROR) {
        return ERROR;
    }

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_CONTROL_MOTOR;
    J60MotorDataToSendCanData(Motor, &Can);

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Run motor in position-velocity-torque mode
 *
 * @param Motor: The pointer to the struct to store motor information
 * @param Position: The position of motor (unit: rad)
 * @param Velocity: The velocity of motor (unit: rad/s)
 * @param Torque: The torque of motor (unit: N*m)
 * @param Kp: The stiffness coefficient of motor (no unit)
 * @param Kd: The damping coefficient of motor (no unit)
 * 
 * @return uint8_t ERROR: Error happened while motor is in position-velocity-torque mode
 *                 NORMAL: It is normal when motor is in position-velocity-torque mode  
 */
uint8_t RunJ60MotorPositonVelocityTorqueMode (MotorInformation* Motor, float Position, float Velocity, float Torque, float Kp, float Kd) {
    if (SetMotionCommand(Motor, CONTROL_MOTOR, Position, Velocity, Kp, Kd, Torque) == ERROR) {
        return ERROR;
    }

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_CONTROL_MOTOR;
    J60MotorDataToSendCanData(Motor, &Can);

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Run motor in different mode
 * 
 * @param Motor: The pointer to the struct to store motor information
 * @param Position: The position of motor (unit: rad)
 * @param Velocity: The velocity of motor (unit: rad/s)
 * @param Torque: The torque of motor (unit: N*m)
 * @param Kp: The stiffness coefficient of motor (no unit)
 * @param Kd: The damping coefficient of motor (no unit)
 * @param Mode: Motor mode
 * 
 * @return uint8_t ERROR: Error happened while motor is running
 *                 NORMAL: It is normal when motor is running 
 */
uint8_t RunJ60Motor (MotorInformation* Motor, float Position, float Velocity, float Torque, float Kp, float Kd, enum MotorMode Mode) {
    if (SetMotionCommand(Motor, CONTROL_MOTOR, 0, Velocity, 0, Kd, Torque) == ERROR) {
        return ERROR;
    }

    Motor->MotorMode = Mode;

    switch (Mode) {
        case PositionMode: 
            if (RunJ60MotorPositionMode(Motor, Position, Kp, Kd) == ERROR) {
                return ERROR;
            }
            break;

        case VelocityMode:
            if (RunJ60MotorVelocityMode(Motor, Velocity, Kd) == ERROR) {
                return ERROR;
            }
            break;

        case TorqueMode:
            if (RunJ60MotorTorqueMode(Motor, Torque) == ERROR) {
                return ERROR;
            }
            break;

        case KdMode: 
            if (RunJ60MotorKdMode(Motor, Kd) == ERROR) {
                return ERROR;
            }
            break;

        case ZeroTorqueMode: 
            if (RunJ60MotorZeroTorqueMode(Motor) == ERROR) {
                return ERROR;
            }
            break;

        case PositionTorqueMode: 
            if (RunJ60MotorPositionTorqueMode(Motor, Position, Torque, Kp, Kd) == ERROR) {
                return ERROR;
            }
            break;
        
        case VelocityTorqueMode: 
            if (RunJ60MotorVelocityTorqueMode(Motor, Velocity, Torque, Kd) == ERROR) {
                return ERROR;
            }
            break;

        case PositionVelocityTorqueMode:
            if (RunJ60MotorPositonVelocityTorqueMode(Motor, Position, Velocity, Torque, Kp, Kd) == ERROR) {
                return ERROR;
            }
            break;

        default: 
            break;
    }

    return NORMAL;
}

/**
 * @brief Get the status word of motor
 * 
 * @param Motor: The pointer to the struct to store motor information
 * 
 * @return uint8_t: ERROR: Error happened while getting status word
 *                  NORMAL: It is normal while getting status word
 */
uint8_t GetJ60MotorStatusWord (MotorInformation* Motor) {
    SetNormalCommand(Motor, GET_STATUS_WORD);

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_GET_STATUS_WORD;

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Reset the error of motor
 * 
 * @param Motor: The pointer to the struct to store motor information
 * 
 * @return uint8_t: ERROR: Error happened while resetting error
 *                  NORMAL: It is normal while resetting error
 */
uint8_t J60MotorErrorReset (MotorInformation* Motor) {
    SetNormalCommand(Motor, ERROR_RESET);

    Can.ID = GetJ60MotorCanID(&Motor->MotorCommand);
    Can.DLC = SEND_DLC_ERROR_RESET;

    if (Can_Send(&Can, &hcan1) == CAN_ERROR) {
        return ERROR;
    }
    return NORMAL;
}

/**
 * @brief Analyse the data returned by motor
 * 
 * @param none
 * 
 * @return none
 */
void AnalyseJ60MotorReceiveData (void) {
    uint8_t ReceiveMotorID = (CanRxInformation.StdId & 0x1F) - 0x10;
    ReceiveCanDataToJ60MotorData(&J60Motor[ReceiveMotorID - 1], &Can);
}
