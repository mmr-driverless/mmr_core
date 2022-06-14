#ifndef INC_MMR_BB_MANAGER_H_
#define INC_MMR_BB_MANAGER_H_

#include "main.h"
#include "mmr_can.h"
#include "mmr_bb_utils.h"
#include <stdbool.h>

#define ADC_SIZE 2
#define DAC_SIZE 1

#define MANUAL 0
#define CLUTCH_INIT 1
#define CLUTCH_WAIT_FIRST 2
#define ECU_WAIT 3
#define RPM_WAIT 4
#define CLUTCH_WAIT_SECOND 5
#define CLUTCH_END 6

#define RPM 6000

#define APPS_MAX 3500
#define APPS_MIN 1000

typedef ADC_HandleTypeDef AdcHandle;
typedef DAC_HandleTypeDef DacHandle;

HalStatus MMR_BB_setup();
HalStatus MMR_BB_initManager(CanHandle *hcan, AdcHandle *hadc, DacHandle *hdac);
HalStatus MMR_BB_startManager();

HalStatus MMR_BB_manualMode();
HalStatus MMR_BB_autonomousMode();

HalStatus MMR_BB_receiveManager();
HalStatus MMR_BB_sendManager();

HalStatus MMR_BB_sendNoData(MmrCanMessageId messageId);
HalStatus MMR_BB_sendAPPS(MmrCanMessageId messageId, uint32_t data); // integer

HalStatus MMR_BB_changeDrivingMode();

#endif /* INC_MMR_BB_MANAGER_H_ */
