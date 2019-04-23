#ifndef GLOBALS_H
#define GLOBALS_H

#include <cstdio>
#include <mutex>
#include <unordered_map>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <pacmod_msgs/PositionWithSpeed.h>
#include <pacmod_msgs/PacmodCmd.h>
#include <pacmod_msgs/VehicleSpeedRpt.h>
#include <pacmod_msgs/SystemRptFloat.h>

#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/accel_cmd.h>
#include <autoware_msgs/brake_cmd.h>
#include <autoware_msgs/steer_cmd.h>

#include <mpc_msgs/Ctrl.h>

// Enums
enum ShiftState
{
  SHIFT_PARK = 2,
  SHIFT_REVERSE = 1,
  SHIFT_NEUTRAL = 2,
  SHIFT_LOW = 3,
  SHIFT_HIGH = 3
};

// Enums
enum ShiftStateAutoware
{
    AUTOWARE_DRIVE = 1,
    AUTOWARE_REVERSE = 2,
    AUTOWARE_BRAKE = 3,
    AUTOWARE_NEUTRAL = 4,
    AUTOWARE_PARK = 4
};

enum TurnSignalState
{
  SIGNAL_RIGHT,
  SIGNAL_OFF,
  SIGNAL_LEFT,
  SIGNAL_HAZARD
};

enum VehicleType
{
  POLARIS_GEM,
  POLARIS_RANGER,
  LEXUS_RX_450H,
  INTERNATIONAL_PROSTAR,
  VEHICLE_4
};

enum JoyButton
{
  TOP_BTN,
  LEFT_BTN,
  BOTTOM_BTN,
  RIGHT_BTN,
  LEFT_BUMPER,
  RIGHT_BUMPER,
  BACK_SELECT_MINUS,
  START_PLUS,
  LEFT_TRIGGER_BTN,   // Sometimes button, sometimes axis
  RIGHT_TRIGGER_BTN,  // Sometimes button, sometimes axis
  LEFT_STICK_PUSH,
  RIGHT_STICK_PUSH
};

struct EnumHash
{
  template <typename T>
  std::size_t operator()(T t) const
  {
    return static_cast<std::size_t>(t);
  }
};

// static constants
//static const float ROT_RANGE_SCALER_LB = 720;
static const float ACCEL_SCALE_FACTOR = 0.6;
static const float ACCEL_OFFSET = 0.21;
static const float STEER_SCALE_FACTOR = 1.5;
static const float STEER_OFFSET = 1.0;
static const float MAX_ROT_RAD_VEHICLE2 = 6.5;
static const float MAX_ROT_RAD_VEHICLE4 = 5.236;
static const float MAX_ROT_RAD_DEFAULT = 10.9956;
static const float AXES_MIN = -1.0;
static const float AXES_MAX = 1.0;
static const float PI = 3.141592653589793238462;
static const uint16_t NUM_WIPER_STATES = 8;
static const uint16_t WIPER_STATE_START_VALUE = 0;
static const uint16_t NUM_HEADLIGHT_STATES = 3;
static const uint16_t HEADLIGHT_STATE_START_VALUE = 0;
static const uint16_t INVALID = -1;
static const uint16_t BUTTON_DOWN = 1;
static const uint16_t ENABLE_INDEX= 5;
static const uint16_t DISABLE_INDEX= 4;

//PACMOD Accel/Brake Limits
static const float ACCEL_MAX = 0.810000017285;
static const float ACCEL_MIN = 0.209999993443;
static const float BRAKE_MAX = 1.0;
static const float BRAKE_MIN = 0.0;
//Brake Delta is used to prevent drifting while making stops
static const float BRAKE_DELTA = 0.01;

// mutex
static std::mutex enable_mutex;
static std::mutex speed_mutex;
static std::mutex accel_mutex;

#endif
