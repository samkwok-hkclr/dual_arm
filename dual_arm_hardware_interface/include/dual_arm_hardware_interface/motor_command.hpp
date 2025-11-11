#ifndef MOTOR_COMMAND_HPP__
#define MOTOR_COMMAND_HPP__

#pragma once

#include <cstdint>

enum MotorADDR : uint8_t 
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
  CUR_CTRL_ID_OFFSET    = 0x400,
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

#endif // MOTOR_COMMAND_HPP__