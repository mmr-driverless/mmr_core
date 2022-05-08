#include "mmr_pid.h"

#define ADC_AVERAGE
#define POSITION_ONLY

#define MAX_VOLTAGE 3.6f
#define MAX_ADC_VALUE 4096.0f
#define VOLTAGE_RATIO 5.0f/3.3f
#define OPEN_CLUTCH_ANGLE 1.145f // [rad]
#define ENGAGED_CLUTCH_ANGLE 0.60f // [rad]

#ifdef POSITION_ONLY
	#define ADC_CHANNELS 2 //number of adc channels
	#define BUFFER_LENGTH 20
#else
	#define ADC_CHANNELS 3 //number of adc channels
	#define BUFFER_LENGTH 21
#endif

typedef uint8_t AdcIndex;
typedef uint8_t BufferLength;
typedef uint16_t AdcValue;

typedef struct ClutchIndexes {
	AdcIndex motorPotentiometer;
	AdcIndex lever;
	#ifndef POSITION_ONLY
		AdcIndex current;
	#endif
} ClutchIndexes;

typedef struct Clutch {
	ClutchIndexes indexes;
	AdcValue *_adcValues;
    float measuredAngle;
    float targetAngle;
	#ifndef POSITION_ONLY
    	float current;
	#endif
} Clutch;

Clutch clutchInit(ClutchIndexes indexes, AdcValue *adcValues);

float getMeasuredAngle(Clutch *clutch);
float getTargetAngle(Clutch *clutch);
float getCurrent(Clutch *clutch);

#ifdef POSITION_ONLY
	float getMotorDutyCycle(Clutch *clutch, PID *pid);
#else
	float getMotorDutyCycle(Clutch *clutch, PID *pid1, PID *pid2);
	float getCurrent(Clutch *clutch);
	float _getCurrentValue(Clutch *Clutch);
#endif

float _getMotorPotentiomerValue(Clutch *Clutch);
float _getLeverValue(Clutch *Clutch);

float _getAvg(Clutch *Clutch, AdcIndex index);
