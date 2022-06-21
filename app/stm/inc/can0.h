#ifndef APP_STM_INC_CAN0_H_
#define APP_STM_INC_CAN0_H_

#include "../../lib/can/inc/can.h"
#include "../../lib/can/inc/message.h"




HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *hcan, CAN_TxHeaderTypeDef *pHeader, uint8_t aData[], uint32_t *pTxMailbox);;

//typedef struct {
//  uint32_t Prescaler;
//  uint32_t Mode;
//  uint32_t SyncJumpWidth;
//  uint32_t TimeSeg1;
//  uint32_t TimeSeg2;
//  FunctionalState TimeTriggeredMode;
//  FunctionalState AutoBusOff;
//  FunctionalState AutoWakeUp;
//  FunctionalState AutoRetransmission;
//  FunctionalState ReceiveFifoLocked;
//  FunctionalState TransmitFifoPriority;
//} CAN_InitTypeDef;
//
//
//typedef struct {
//  uint32_t StdId;
//  uint32_t ExtId;
//  uint32_t IDE;
//  uint32_t RTR;
//  uint32_t DLC;
//  FunctionalState TransmitGlobalTime;
//} CAN_TxHeaderTypeDef;
//
//
//typedef struct {
//  uint32_t StdId;
//  uint32_t ExtId;
//  uint32_t IDE;
//  uint32_t RTR;
//  uint32_t DLC;
//  uint32_t Timestamp;
//  uint32_t FilterMatchIndex;
//} CAN_RxHeaderTypeDef;

#endif // !APP_STM_INC_CAN0_H_
