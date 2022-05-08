#include "mmr_pid.h"

#define MAX_VOLTAGE 3.6f
#define MAX_ADC_VALUE 4096.0f
#define VOLTAGE_RATIO 5.0f/3.3f
#define OPEN_CLUTCH_ANGLE 1.145f // [rad]
#define ENGAGED_CLUTCH_ANGLE 0.60f // [rad]

typedef struct ClutchIndexes {
  const uint8_t motorPotentiometer;
  const uint8_t lever;
  const uint8_t current;
} ClutchIndexes;

typedef struct Clutch {
  const ClutchIndexes indexes;
  const uint8_t bufferLength;
  float measuredAngle;
  float targetAngle;
  uint16_t *_adcValues;
} Clutch;

Clutch ClutchInit(ClutchIndexes indexes, uint16_t *adcValues, uint8_t bufferLength);

float getMeasuredAngle(Clutch *clutch);
float getTargetAngle(Clutch *clutch);

float getMotorDutyCycle(Clutch *clutch, PID *pid);

float _getMotorPotentiomerValue(Clutch *Clutch);
float _getLeverValue(Clutch *Clutch);
float _getCurrentValue(Clutch *Clutch);