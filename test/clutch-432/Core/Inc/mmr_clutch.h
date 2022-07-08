#include "mmr_pid.h"
#include "mmr_lowpass.h"

#define ADC_AVERAGE

#define MAX_VOLTAGE 3.6f
#define MAX_ADC_VALUE 4096.0f
#define VOLTAGE_RATIO (5.0f/3.3f)

#define OPEN_CLUTCH_ANGLE (0.69f)//(0.83f+0.025f) // [rad]
#define ENGAGED_CLUTCH_ANGLE (1.62f) // [rad]

#define OPEN_LEVER_ANGLE 0.60f // [rad] // 1350
#define ENGAGED_LEVER_ANGLE 1.02f // [rad] // 2160

#define ADC_CHANNELS 2//number of adc channels
#define BUFFER_LENGTH 20

enum DrivingMode {
	MANUAL,
	AUTONOMOUS,
};

enum ErrorState {
	OK,
	POTENTIOMETER_OUT_OF_RANGE,
};

typedef enum DrivingMode DrivingMode;
typedef enum ErrorState ErrorState;
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
	ErrorState state;
	bool inProgress;
	LowpassData _lpDataMeasured;
} Clutch;

Clutch clutchInit(ClutchIndexes indexes, ClutchPID clutchPID, AdcValue *adcValues);
void openClutch(Clutch *clutch);
void engagedClutch(Clutch *clutch);

void setDrivingMode(Clutch *clutch, DrivingMode mode);
void setErrorState(Clutch *clutch, ErrorState state);
void setTargetAngle(Clutch *clutch, float angle);

float getPotMotAngle(Clutch *clutch);
float getLeverAngle(Clutch *clutch);
float getCurrent(Clutch *clutch);

float getMotorDutyCycle(Clutch *clutch);

AdcValue _getMotorPotentiomerValue(Clutch *Clutch);
AdcValue _getLeverValue(Clutch *Clutch);
float _getPotentiometerAngle(AdcValue value, Clutch *Clutch);

