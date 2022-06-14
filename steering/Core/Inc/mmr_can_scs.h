/**
 * @file mmr_can_scs_manager.h
 * @brief
 * Provides utilities for managing the scs messages,
 * such as transmission, retransmission and timeout error.
 */

#ifndef INC_MMR_CAN_SCS_H_
#define INC_MMR_CAN_SCS_H_

#include "mmr_can.h"


/**
 * @brief
 * The maximum number of scs messages that
 * can be tracked at any given time.
 */
#ifndef MMR_CAN_SCS_ENTRIES_COUNT
#define MMR_CAN_SCS_ENTRIES_COUNT 5
#endif


/**
 * @brief 
 * Maximum timeout before first retransmission,
 * in milliseconds 
 */
#ifndef MMR_CAN_MAX_TIMEOUT
#define MMR_CAN_MAX_TIMEOUT 500
#endif


/**
 * @brief
 * It depends on how many bits the board devotes to the timer
 * Check the datasheet
 */
typedef uint32_t TimerRange;


/**
 * @brief
 * Represents the base-struct to manage a single RTR
 * and allows to interface with the associated SCS's timer
 */
typedef struct MmrCanScsEntry {
  /**
   * @brief
   * The header used to index this entry.
   */
  MmrCanHeader header;

  /**
   * @brief
   * The time at which this message was sent,
   * represented as milliseconds since the board
   * was turned on.
   */
  TimerRange counter;

  /**
   * @brief
   * Number of retransmissions for this message.
   * 
   * == 0 -> No retransmission occurred.
   * >= 1 -> The scs was retransmitted.
   */
  int rtr;
} MmrCanScsEntry;


/**
 * @brief
 * The results of an scsCheck operation.
 */
typedef enum MmrCanScsCheckResult {
  /**
   * @brief No timeout error.
   */
  MMR_CAN_SCS_CHECK_OK,

  /**
   * @brief The message should be retransmitted.
   */
  MMR_CAN_SCS_CHECK_RTR,

  /**
   * @brief
   * The message has timed out and has already been
   * retransmitted, fail.
   */
  MMR_CAN_SCS_CHECK_ERROR,
} MmrCanScsCheckResult;


/**
 * @brief 
 * Tries to handle an acknowledgment for a particular scs.
 * 
 * This function MUST be called every time a
 * message is received, as it might potentially be an
 * ACK.
 * 
 * @param header The header to check.
 * @return true The message was an ACK and was cleared accordingly.
 * @return false The message wasn't an ACK.
 */
bool MMR_CAN_MaybeHandleAck(MmrCanHeader header);

/**
 * @brief
 * Checks the array with the stored scs messages and
 * retransmits the message if no ack was received.
 * 
 * This function MUST be called at every loop.
 * 
 * @return HalStatus
 *  The result of the operation.
 *  HAL_ERROR should immediately be handled as a safe state.
 */
HalStatus MMR_CAN_HandleNextScs(CanHandle *hcan);

/**
 * @brief
 * Send an acknowledgment packet based on the
 * given scs header.
 * 
 * @param originalHeader The scs header to acknowledge.
 */
HalStatus MMR_CAN_SendAck(
  CanHandle *hcan,
  MmrCanHeader originalHeader
);

/**
 * @brief
 * Sends an Scs message.
 * 
 * @param scsId The MMR_CAN_MESSAGE_ID_SCS_xx id.
 * @param senderId The id of this board.
 */
HalStatus MMR_CAN_SendScs(
  CanHandle *hcan,
  MmrCanMessageId scsId,
  CanId senderId
);


MmrCanScsEntry* MMR_CAN_GetNextScsEntry();
MmrCanScsEntry* MMR_CAN_PutScsEntry(MmrCanHeader header);
MmrCanScsEntry* MMR_CAN_ClearScsEntry(MmrCanHeader header);
MmrCanScsEntry* MMR_CAN_FindScsEntry(MmrCanHeader header);

#endif // !INC_MMR_CAN_SCS_H_
