/**
 * @file mmr_can_events.h
 * @brief
 * This header provides a set of utilities for working
 * with interrupts.
 * 
 * The recommended way of readings the CAN bus is
 * via polling, altought interrupt may be used for
 * monitoring critical messages.
 */

#ifndef INC_MMR_CAN_EVENTS_H_
#define INC_MMR_CAN_EVENTS_H_

#include "mmr_can.h"
#include "mmr_can_util.h"

typedef void (*MmrCanEventHandler)(const MmrCanMessage *event);

typedef struct MmrCanEventList {
  const MmrCanEventHandler *handlers;
  const size_t count;
} MmrCanEventList;

#define MMR_CAN_CreateEventList(handlers) \
  (const MmrCanEventList) { handlers, sizeofarray(handlers) }


/**
 * @brief
 * Activates the CAN rx interrupts
 *
 * When one is fired, the callbacks provided inside the
 * MmrCanEventList will be invoked
 */
HalStatus MMR_CAN_InitRxHandlers(CanHandle *hcan, const MmrCanEventList *rxEvents);


#endif /* INC_MMR_CAN_EVENTS_H_ */
