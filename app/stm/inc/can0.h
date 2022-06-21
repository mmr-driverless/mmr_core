#ifndef APP_STM_INC_CAN0_H_
#define APP_STM_INC_CAN0_H_

#include "../../lib/can/inc/can.h"
#include "../../lib/can/inc/message.h"


extern MmrCan can0;

#if !defined(CAN)
#include "stm_hal_can_defs.h"
#endif

#endif // !APP_STM_INC_CAN0_H_
