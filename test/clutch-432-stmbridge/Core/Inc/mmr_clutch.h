#include "mmr_pid.h"
#include "mmr_lowpass.h"

#define ADC_AVERAGE

#define MAX_VOLTAGE 3.6f
#define MAX_ADC_VALUE 4096.0f
#define VOLTAGE_RATIO (5.0f/3.3f)

#define OPEN_CLUTCH_ANGLE 0.57f // [rad]
#define ENGAGED_CLUTCH_ANGLE 1.7f // [rad]

#define OPEN_LEVER_ANGLE 0.65f // [rad]
#define ENGAGED_LEVER_ANGLE 1.1f // [rad]

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
	AdcIndex current;
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
    float current;
	DrivingMode mode;
	bool inProgress;
	LowpassData _lpDataMeasured;
} Clutch;

Clutch clutchInit(ClutchIndexes indexes, ClutchPID clutchPID, AdcValue *adcValues);
void openClutch(Clutch *clutch);
void engagedClutch(Clutch *clutch);

void setDrivingMode(Clutch *clutch, DrivingMode mode);
void setTargetAngle(Clutch *clutch, float angle);

float getPotMotAngle(Clutch *clutch);
float getLeverAngle(Clutch *clutch);
float getCurrent(Clutch *clutch);

float getMotorDutyCycle(Clutch *clutch);

AdcValue _getMotorPotentiomerValue(Clutch *Clutch);
AdcValue _getLeverValue(Clutch *Clutch);
float _getPotentiometerAngle(AdcValue value, Clutch *Clutch);

