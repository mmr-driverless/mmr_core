#ifndef APP_CORE_BACK_INC_MISSION_H_
#define APP_CORE_BACK_INC_MISSION_H_


typedef enum MmrMission {
  MMR_MISSION_IDLE,
  MMR_MISSION_ACCELERATION,
  MMR_MISSION_SKIDPAD,
  MMR_MISSION_AUTOCROSS,
  MMR_MISSION_TRACKDRIVE,
  MMR_MISSION_EBS_TEST,
  MMR_MISSION_INSPECTION,
  MMR_MISSION_MANUAL,
  MMR_MISSION_FINISHED,
} MmrMission;

MmrMission MMR_MISSION_Run(MmrMission mission);

#endif // !APP_CORE_BACK_INC_MISSION_H_
