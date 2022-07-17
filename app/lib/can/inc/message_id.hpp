/**
 * @file mmr_can_message_id.h
 * @brief
 * This header contains the message id declarations.
 * 
 * Message ids identify a message, allowing the receiver
 * to take appropriate action when parsing one.
 * 
 * For example, a can packet with message id set to
 * MMR_CAN_MESSAGE_ID_POINT might be interpreted as
 * a message carrying a struct Point { int x; int y; };,
 * and thus deserialized accordingly.
 *
 * You can find a table with more in-depth explanations on
 * Google Drive:
 * https://docs.google.com/spreadsheets/d/1GIC0_FuhCjXfBOg_dUWzoEsl5TG7SEOR
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

using MessageId = uint16_t;

struct MessageIds {
  enum Res {
    RES_NMT = 0,
    RES = 0x191,
  };

  enum S {
    LV12 = 1,
    LV24,
    PF,
    FRBPS,
    EBS,
    APPS,
    TPS,
    CLUTCH,
    PPA,
  };
  
  enum AS {
    R2D = 32,
    TS,
    FINISHED,
    EMERGENCY,
    READY,
    DRIVING,
    OFF,
    ASB_CHECK,
    RES_EMERGENCY,
    EBS_FLAG,
    ASB_ENGAGED,
    VEHICLE_STANDSTILL,
    AS_STATE,
  };
  
  enum M {
    SELECTED = 64,
    READY,
    FINISHED,
  };
  
  enum D {
    STEERING_ANGLE = 138,
    BREAKING_PERCENTAGE,
    ACCELERATOR_PERCENTAGE,
    SPEED_ODOMETRY,
    MOTOR_MOMENT_TARGET,
    SPEED_TARGET,
    STEERING_STATE,
    LAP_COUNTER,
    CONES_COUNT_ACTUAL,
    CONES_COUNT_ALL,
    AS_CHANGE_MODE,
  };
  
  enum ECU {
    PEDAL_THROTTLE = 192,
    TEMPERATURES,
    ENGINE_FN1,
    PRESSURES,
    ENGINE_FN2,
    LAMBDA,
    WHEEL_SPEEDS,
    BRAKE_PRESSURES,
    EBS_PRESSURES,
  };
  
  enum CS {
    TS_EBS = 222,
    ASB_STATE,
    CLUTCH_PULL,
    CLUTCH_PULL_OK,
    CLUTCH_RELEASE,
    CLUTCH_RELEASE_OK,
    CLUTCH_SET_MANUAL, 
  };
  
  enum ST {
    ERROR_LEFT_X = 256,
    ERROR_RIGHT_X,
    ODOMETRY_MIN_SPEED_LEFT_X,
    ODOMETRY_MIN_SPEED_LEFT_Y,
    ODOMETRY_MIN_SPEED_RIGHT_X,
    ODOMETRY_MIN_SPEED_RIGHT_Y,
    ODOMETRY_MAX_SPEED_LEFT_Y,
    ODOMETRY_MAX_SPEED_RIGHT_Y,
  };
  
  enum BRK {
    PROPORTIONAL_ERROR_LEFT_X	= 288,
    PROPORTIONAL_ERROR_LEFT_Y,
    PROPORTIONAL_ERROR_RIGHT_X,
    PROPORTIONAL_ERROR_RIGHT_Y,
    RISE_CUTOFF_FREQUENCY,
    ZERO_PRESSURE_AUTOSET,
    ZERO_PRESSURE_AUTOSET_OK,
    CURRENT_PRESSURE,
    MAX_PRESSURE,
    TARGET_PRESSURE,
    CHECK_ASB_STATE,
  };
};
