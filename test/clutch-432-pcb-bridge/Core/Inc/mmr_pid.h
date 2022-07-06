#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"

#define MAGIC_K 1.0f

// #define INVERT_ERROR

typedef struct PIDSaturation {
  float max;
  float min;
} PIDSaturation;

typedef struct PIDParameters {
  float p;
  float i;
  float d;
} PIDParameters;

typedef struct PIDOutputs {
  float output;
  float outputPresaturation;
} PIDOutputs;

typedef struct PID {
  PIDSaturation saturation;
  PIDParameters parameters;

  float sampleTime;
  float tau;

  PIDParameters _terms;
  PIDOutputs _lastOutputs;
  float _lastError;
} PID;

PID PIDInit(PIDSaturation saturation, PIDParameters parameters, float sampleTime, float tau);
float PIDCompute(PID* pid, float reference, float measured);
float PIDCascade(PID* pid1, PID* pid2, float reference, float measured, float measured2);

float getProportionalTerm(PID* pid, float error);
float getIntegralTerm(PID* pid, float error);
float getDerivativeTerm(PID* pid, float error);

float getError(float reference, float measured);
float getOutput(PID* pid);

bool _isLastOutputSaturated(PID* pid);
void _updateTerms(PID* pid, float error);

void setDirection(float error);
void setClockwise();
void setCounterClockwise();
void resetDir();
