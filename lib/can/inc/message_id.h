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

/**
 * @brief
 * Message ids are 10 bits wide
 * The first 5 bits determine the message id type, while
 * the other 5 determine the subtype.
 * 
 * Ids scheme:
 *
 *   id                        | type  | subtype
 * ----------------------------|-------|---------
 *  SENSING                    | 00000 | xxxxx
 *  AUTONOMOUS_STATE           | 00001 |
 *  MANUAL_MISSION             | 00010 |
 *  AUTONOMOUS_MISSION         | 00011 |
 *  AUTONOMOUS_MISSION_CONTROL | 00100 |
 *  DRIVE                      | 00101 |
 *  ECU_BOSCH                  | 00110 |
 */
typedef enum MmrCanMessageId MmrCanMessageId;

typedef enum MmrCanMessageIdType {
  MMR_CAN_MESSAGE_ID_TYPE_SENSING,
  MMR_CAN_MESSAGE_ID_TYPE_AUTONOMOUS_STATE,
  MMR_CAN_MESSAGE_ID_TYPE_MANUAL_MISSION,
  MMR_CAN_MESSAGE_ID_TYPE_AUTONOMOUS_MISSION,
  MMR_CAN_MESSAGE_ID_TYPE_AUTONOMOUS_MISSION_CONTROL,
  MMR_CAN_MESSAGE_ID_TYPE_DRIVE,
  MMR_CAN_MESSAGE_ID_TYPE_ECU_BOSCH,
} MmrCanMessageIdType;


/**
 * @brief
 * Returns the 3 bits representing
 * the MmrCanMessageIdType.
 */
uint8_t MMR_CAN_GetMessageIdType(MmrCanMessageId msgId);

/**
 * @brief
 * Returns the 7 bits representing
 * the message id's subtype.
 */
uint8_t MMR_CAN_GetMessageIdSubtype(MmrCanMessageId msgId);

/**
 * @brief
 * Tells wether the provided message
 * is of the given id type.
 *
 * E.g. if a message is an SCS.
 */
bool MMR_CAN_IsMessageIdOfType(MmrCanMessageId msgId, MmrCanMessageIdType type);


enum MmrCanMessageId {
// SENSING
  MMR_CAN_MESSAGE_ID_S_PPA = 0x0,
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
// !AUTONOMOUS_STATE
// MANUAL_MISSION
  MMR_CAN_MESSAGE_ID_MM_MANUAL = 64,
// !MANUAL_MISSION
// AUTONOMOUS_MISSION
  MMR_CAN_MESSAGE_ID_AM_ACCELERATION = 96,
  MMR_CAN_MESSAGE_ID_AM_SKIDPAD,
  MMR_CAN_MESSAGE_ID_AM_AUTOCROSS,
  MMR_CAN_MESSAGE_ID_AM_TRACKDRIVE,
  MMR_CAN_MESSAGE_ID_AM_EBS_TEST,
  MMR_CAN_MESSAGE_ID_AM_INSPECTION,
// !AUTONOMOUS_MISSION
// AUTONOMOUS_MISSION_CONTROL
  MMR_CAN_MESSAGE_ID_AMC_MISSION_FINISHED = 128,
  MMR_CAN_MESSAGE_ID_AMC_VEHICLE_STANDSTILL,
  MMR_CAN_MESSAGE_ID_AMC_MISSION_SELECTED,
  MMR_CAN_MESSAGE_ID_AMC_ASMS,
  MMR_CAN_MESSAGE_ID_AMC_ASB,
  MMR_CAN_MESSAGE_ID_AMC_TS,
  MMR_CAN_MESSAGE_ID_AMC_BE,
// !AUTONOMOUS_MISSION_CONTROL
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
// !STEERING
};

#endif // !INC_MMR_CAN_MESSAGE_ID_H_
