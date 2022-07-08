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

#ifndef INC_MMR_CAN_MESSAGE_ID_H_
#define INC_MMR_CAN_MESSAGE_ID_H_

#include <stdint.h>
#include <stdbool.h>
#include "../../util/inc/binary_literals.h"

typedef enum MmrCanMessageId {
// SENSING
  MMR_CAN_MESSAGE_ID_S_PPA = 0,
  MMR_CAN_MESSAGE_ID_S_LV12,
  MMR_CAN_MESSAGE_ID_S_LV24,
  MMR_CAN_MESSAGE_ID_S_PF,
  MMR_CAN_MESSAGE_ID_S_FRBPS,
  MMR_CAN_MESSAGE_ID_S_EBS,
  MMR_CAN_MESSAGE_ID_S_APPS,
  MMR_CAN_MESSAGE_ID_S_TPS,
  MMR_CAN_MESSAGE_ID_S_CLUTCH,
// !SENSING
// AUTONOMOUS_STATE
  MMR_CAN_MESSAGE_ID_AS_R2D = 32,
  MMR_CAN_MESSAGE_ID_AS_TS,
  MMR_CAN_MESSAGE_ID_AS_FINISHED,
  MMR_CAN_MESSAGE_ID_AS_EMERGENCY,
  MMR_CAN_MESSAGE_ID_AS_READY,
  MMR_CAN_MESSAGE_ID_AS_DRIVING,
  MMR_CAN_MESSAGE_ID_AS_OFF,
  MMR_CAN_MESSAGE_ID_AS_ASB_CHECK,
// !AUTONOMOUS_STATE
// MANUAL_MISSION
  MMR_CAN_MESSAGE_ID_M_MISSION_SELECTED = 64,
// !MANUAL_MISSION
// DRIVE
  MMR_CAN_MESSAGE_ID_D_STEERING_ANGLE = 138,
  MMR_CAN_MESSAGE_ID_D_BREAKING_PERCENTAGE,
  MMR_CAN_MESSAGE_ID_D_ACCELERATOR_PERCENTAGE,
  MMR_CAN_MESSAGE_ID_D_SPEED_ODOMETRY,
// !DRIVE
// ECU_BOSCH
  MMR_CAN_MESSAGE_ID_ECU_PEDAL_THROTTLE = 192,
  MMR_CAN_MESSAGE_ID_ECU_TEMPERATURES,
  MMR_CAN_MESSAGE_ID_ECU_ENGINE_FN1,
  MMR_CAN_MESSAGE_ID_ECU_PRESSURES,
  MMR_CAN_MESSAGE_ID_ECU_ENGINE_FN2,
  MMR_CAN_MESSAGE_ID_ECU_LAMBDA,
  MMR_CAN_MESSAGE_ID_ECU_WHEEL_SPEEDS,
  MMR_CAN_MESSAGE_ID_ECU_BRAKE_PRESSURES,
// !ECU_BOSCH
// CONTROL_SIGNAL
  MMR_CAN_MESSAGE_ID_CS_CLUTCH_PULL = 224,
  MMR_CAN_MESSAGE_ID_CS_CLUTCH_PULL_OK,
  MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE,
  MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE_OK,
  MMR_CAN_MESSAGE_ID_CS_CLUTCH_SET_MANUAL,
// !CONTROL_SIGNAL
// STEERING
  MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ERROR_LEFT_X = 256,
  MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ERROR_RIGHT_X,
  MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MIN_SPEED_LEFT_X,
  MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MIN_SPEED_LEFT_Y,
  MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MIN_SPEED_RIGHT_X,
  MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MIN_SPEED_RIGHT_Y,
  MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MAX_SPEED_LEFT_Y,
  MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MAX_SPEED_RIGHT_Y,
  MMR_CAN_MESSAGE_ID_ST_RISE_CUTOFF_FREQUENCY,
  MMR_CAN_MESSAGE_ID_ST_STEERING_CENTER_AUTOSET,
  MMR_CAN_MESSAGE_ID_ST_STEERING_CENTER_AUTOSET_OK,
  MMR_CAN_MESSAGE_ID_ST_CURRENT_ANGLE,
// !STEERING

// BRAKE
  MMR_CAN_MESSAGE_ID_BRK_PROPORTIONAL_ERROR_LEFT_X = 288,
  MMR_CAN_MESSAGE_ID_BRK_PROPORTIONAL_ERROR_LEFT_Y,
  MMR_CAN_MESSAGE_ID_BRK_PROPORTIONAL_ERROR_RIGHT_X,
  MMR_CAN_MESSAGE_ID_BRK_PROPORTIONAL_ERROR_RIGHT_Y,
  MMR_CAN_MESSAGE_ID_BRK_RISE_CUTOFF_FREQUENCY,
  MMR_CAN_MESSAGE_ID_BRK_ZERO_PRESSURE_AUTOSET,
  MMR_CAN_MESSAGE_ID_BRK_ZERO_PRESSURE_AUTOSET_OK,
  MMR_CAN_MESSAGE_ID_BRK_CURRENT_PRESSURE,
  MMR_CAN_MESSAGE_ID_BRK_MAX_PRESSURE,
  MMR_CAN_MESSAGE_ID_BRK_TARGET_PRESSURE,
  MMR_CAN_MESSAGE_ID_BRK_CHECK_ASB_STATE
// !BRAKE
} MmrCanMessageId;

#endif // !INC_MMR_CAN_MESSAGE_ID_H_
