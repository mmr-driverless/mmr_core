#ifndef APP_BACK_INC_AS_H_
#define APP_BACK_INC_AS_H_

#include <can.h>
#include <pin.h>
#include "timing.h"
#include "delay.h"


typedef enum MmrAsState {
  AS_IDLE,
  AS_OFF,
  AS_READY,
  AS_DRIVING,
  AS_EMERGENCY,
  AS_FINISHED,
} MmrAsState;

typedef enum asStatus {
	OFF = 0U,
	ON,
} asStatus;

void MMR_AS_Init(MmrCan *can, MmrPin *gearUp, MmrPin *gearDown, MmrPin *gearN, MmrPin *changeMode, uint32_t *apps, uint32_t *adc);
void Buzzer_activation(void);
void Buzzer_disactivation(void);

MmrMission MMR_AS_Run(MmrMission mission);

#endif // !APP_BACK_INC_AS_H_
