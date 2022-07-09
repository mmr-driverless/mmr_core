 #ifndef APP_BACK_INC_AS_H_
#define APP_BACK_INC_AS_H_

#include <can.h>
#include <pin.h>
#include "timing.h"
#include "delay.h"


typedef struct MmrGlobalState MmrGlobalState;


typedef enum MmrLaunchControlMode {
//  MMR_AS_MODE_IDLE = 0,
  MMR_AS_MODE_MANUAL = 0,
  MMR_AS_MODE_GEAR_CHANGE = 1,
  MMR_AS_MODE_AUTONOMOUS = 2,
} MmrLaunchControlMode;


typedef enum MmrMission {
  MMR_MISSION_IDLE,
  MMR_MISSION_ACCELERATION,
  MMR_MISSION_SKIDPAD,
  MMR_MISSION_AUTOCROSS,
  MMR_MISSION_TRACKDRIVE,
  MMR_MISSION_EBS_TEST,
  MMR_MISSION_INSPECTION,
  MMR_MISSION_MANUAL,
} MmrMission;

typedef enum MmrAsState {
  AS_IDLE,
  AS_OFF,
  AS_READY,
  AS_DRIVING,
  AS_EMERGENCY,
  AS_FINISHED,
} MmrAsState;

typedef enum asStatus
{
	OFF = 0U,
	ON,
}asStatus;
void MMR_AS_Init(MmrCan *can, MmrPin *gearUp, MmrPin *gearDown, MmrPin *gearN, MmrPin *changeMode, uint32_t *apps, uint32_t *adc);
void Buzzer_activation(void);
void Buzzer_disactivation(void);

MmrLaunchControlMode MMR_AS_Run(MmrLaunchControlMode mode);

#endif // !APP_BACK_INC_AS_H_
