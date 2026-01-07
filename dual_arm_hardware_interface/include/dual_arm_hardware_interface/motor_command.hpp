#ifndef MOTOR_COMMAND_HPP__
#define MOTOR_COMMAND_HPP__

#pragma once

#include <cstdint>

enum MotorAddr : uint8_t 
{
  CLEAR_ERROR           = 0x0F,
  WORK_MODE             = 0x30,
  IAP_FLAG              = 0x49,
  ENABLE_FLAG           = 0x0A,
};

enum CanIdOffset : uint32_t
{
  IAP_FLAG_ID_OFFSET    = 0x100,
  POS_CTRL_ID_OFFSET    = 0x200,
  VEL_CTRL_ID_OFFSET    = 0x300,
  CURR_CTRL_ID_OFFSET   = 0x400,
  SERVO_RESP_ID_OFFSET  = 0x500,
  STATUS_REQ_ID_OFFSET  = 0x600,
  STATUS_RESP_ID_OFFSET = 0x700,
};

enum MotorMode : uint8_t
{
  EFFORT_MODE   = 0x01,
  VELOCITY_MODE = 0x02,
  POSITION_MODE = 0x03,
};

enum MotorRegister : uint8_t
{
  SYSTEM_VOLTAGE = 0x05,
  SYSTEM_TEMP    = 0x06, 
  CURRENT_P      = 0x51,
  CURRENT_I      = 0x52,
  CURRENT_D      = 0x53,
  VELOCITY_P     = 0x54,
  VELOCITY_I     = 0x55,
  VELOCITY_D     = 0x56,
  POSITION_P     = 0x58,
  POSITION_I     = 0x59,
  POSITION_D     = 0x5A,
};

enum ErrorCode : uint16_t 
{
  HIGH_FOC_RATE             = 0x0001,  // FOC rate too high
  OVER_VOLTAGE              = 0x0002,  // Over voltage
  UNDER_VOLTAGE             = 0x0004,  // Under voltage
  OVER_TEMPERATURE          = 0x0008,  // Over temperature
  STARTUP_FAILED            = 0x0010,  // Startup failed
  ENCODER_OR_ZERO_LOST      = 0x0020,  // Encoder error/Zero position lost
  OVER_CURRENT              = 0x0040,  // Over current
  SOFTWARE_ERROR            = 0x0080,  // Software error
  TEMP_SENSOR_ERROR         = 0x0100,  // Temperature sensor error
  POSITION_LIMIT_EXCEEDED   = 0x0200,  // Position limit exceeded
  INVALID_JOINT_ID          = 0x0400,  // Invalid CAN ID
  POSITION_TRACKING_ERROR   = 0x0800,  // Position tracking error exceeded threshold
  CURRENT_SENSOR_ERROR      = 0x1000,  // Current sensor detection error
  BRAKE_FAILURE             = 0x2000,  // Brake failure
  POSITION_COMMAND_STEP     = 0x4000,  // Position command step warning
  MULTI_TURN_LOSS           = 0x8000   // Multi-turn count loss
};

std::string error_code_to_str(uint16_t errorCode) 
{
  switch (errorCode) 
  {
    case HIGH_FOC_RATE:           return "HIGH_FOC_RATE";
    case OVER_VOLTAGE:            return "OVER_VOLTAGE";
    case UNDER_VOLTAGE:           return "UNDER_VOLTAGE";
    case OVER_TEMPERATURE:        return "OVER_TEMPERATURE";
    case STARTUP_FAILED:          return "STARTUP_FAILED";
    case ENCODER_OR_ZERO_LOST:    return "ENCODER_OR_ZERO_LOST";
    case OVER_CURRENT:            return "OVER_CURRENT";
    case SOFTWARE_ERROR:          return "SOFTWARE_ERROR";
    case TEMP_SENSOR_ERROR:       return "TEMP_SENSOR_ERROR";
    case POSITION_LIMIT_EXCEEDED: return "POSITION_LIMIT_EXCEEDED";
    case INVALID_JOINT_ID:        return "INVALID_JOINT_ID";
    case POSITION_TRACKING_ERROR: return "POSITION_TRACKING_ERROR";
    case CURRENT_SENSOR_ERROR:    return "CURRENT_SENSOR_ERROR";
    case BRAKE_FAILURE:           return "BRAKE_FAILURE";
    case POSITION_COMMAND_STEP:   return "POSITION_COMMAND_STEP";
    case MULTI_TURN_LOSS:         return "MULTI_TURN_LOSS";
    default:                      return "UNKNOWN_ERROR";
  }
}

#endif // MOTOR_COMMAND_HPP__