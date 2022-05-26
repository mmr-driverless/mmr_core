#include "mmr_pid.h"

#define ADC_AVERAGE

#define MAX_VOLTAGE 3.6f
#define MAX_ADC_VALUE 4096.0f
#define VOLTAGE_RATIO (5.0f/3.3f)
#define OPEN_CLUTCH_ANGLE 0.14f // [rad]
#define ENGAGED_CLUTCH_ANGLE 1.03f // [rad]

#define OPEN_LEVER_ANGLE 2.3f // [rad]
#define ENGAGED_LEVER_ANGLE 3.3f // [rad]

#define ADC_CHANNELS 2//number of adc channels
#define BUFFER_LENGTH 20

enum DrivingMode {
	MANUAL,
	AUTONOMOUS,
};

typedef enum DrivingMode DrivingMode;
typedef uint8_t AdcIndex;
typedef uint8_t BufferLength;
typedef uint16_t AdcValue;

typedef struct ClutchIndexes {
	AdcIndex motorPotentiometer;
	AdcIndex lever;
} ClutchIndexes;

typedef struct ClutchPID {
	PID *pid1;
	PID *pid2;
} ClutchPID;

typedef struct Clutch {
	ClutchPID clutchPID;
	ClutchIndexes indexes;
	AdcValue *_adcValues;
    float measuredAngle;
    float targetAngle;
	DrivingMode mode;
} Clutch;

Clutch clutchInit(ClutchIndexes indexes, ClutchPID clutchPID, AdcValue *adcValues);
void setDrivingMode(Clutch *clutch, DrivingMode mode);
void setTargetAngle(Clutch *clutch, float angle);

float getPotMotAngle(Clutch *clutch);
float getLeverAngle(Clutch *clutch);

float getMotorDutyCycle(Clutch *clutch);

float _getMotorPotentiomerValue(Clutch *Clutch);
float _getLeverValue(Clutch *Clutch);
