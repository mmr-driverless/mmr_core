/**
 * @file mmr_can.h
 * @brief
 * Main header for the mmr_can library.
 */

#ifndef INC_MMR_CAN_H_
#define INC_MMR_CAN_H_

#include <stdint.h>
#include <stdbool.h>
#include "../../util/inc/binary_literals.h"
#include "header.h"
#include "types.h"
#include "optimize.h"
#include "scs.h"

#ifndef MMR_CAN_RX_FIFO
#define MMR_CAN_RX_FIFO CAN_RX_FIFO0
#endif

#if MMR_CAN_RX_FIFO == CAN_RX_FIFO0
#define MMR_CAN_FILTER_FIFO CAN_FILTER_FIFO0
#else
#define MMR_CAN_FILTER_FIFO CAN_FILTER_FIFO1
#endif

#if MMR_CAN_RX_FIFO == CAN_RX_FIFO0
#define MMR_CAN_RX_INTERRUPT CAN_IT_RX_FIFO0_MSG_PENDING
#else
#define MMR_CAN_RX_INTERRUPT CAN_IT_RX_FIFO1_MSG_PENDING
#endif

#ifndef MMR_CAN_MAX_DATA_LENGTH
#define MMR_CAN_MAX_DATA_LENGTH 8
#endif


typedef uint32_t (*MmrCanTickProvider)();


/**
 * @brief
 * A buffer large enough to hold a CAN payload.
 */
typedef uint8_t CanRxBuffer[MMR_CAN_MAX_DATA_LENGTH];


typedef struct Can {
  bool (*trySend)(uint8_t *buffer, int len);
} Can;


typedef struct MmrCanFilterSettings {
  bool enabled;
  CanFilterFifo fifo;
  CanFilterBank bank;
  CanFilterBank slaveBankStart;
  CanFilterMask idMask;
} MmrCanFilterSettings;


typedef struct MmrCanPacket {
  MmrCanHeader header;
  uint8_t *data;
  uint8_t length;
  bool noExtId;
} MmrCanPacket;


typedef struct MmrCanMessage {
  MmrCanHeader header;
  void *store;
} MmrCanMessage;


#define MMR_CAN_FilterConfigDefault(phcan) \
  MMR_CAN_FilterConfig(phcan, MMR_CAN_GetDefaultFilterSettings())


extern MmrCanTickProvider __mmr_can_tickProvider;


void MMR_CAN_SetTickProvider(MmrCanTickProvider tickProvider);
uint32_t MMR_CAN_GetCurrentTick();
HalStatus MMR_CAN_BasicSetupAndStart(CanHandle *hcan);
HalStatus MMR_CAN_FilterConfig(CanHandle *hcan, MmrCanFilterSettings settings);
MmrCanFilterSettings MMR_CAN_GetDefaultFilterSettings();
HalStatus MMR_CAN_Send(CanHandle *hcan, MmrCanPacket packet);
HalStatus MMR_CAN_SendNoTamper(CanHandle *hcan, MmrCanPacket packet);
MmrResult MMR_CAN_TryReceive(CanHandle *hcan, MmrCanMessage *result);
HalStatus MMR_CAN_Receive(CanHandle *hcan, MmrCanMessage *result);

#endif /* INC_MMR_CAN_H_ */
