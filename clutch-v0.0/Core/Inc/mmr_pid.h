#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

typedef struct PIDSaturation {
  const float max;
  const float min;
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
  const PIDSaturation saturation;
  const PIDParameters parameters;

	const float sampleTime;
	const float tau;

  PIDParameters _terms;
  PIDOutputs _lastOutputs;
	float _lastError;
} PID;

PID PIDInit(PIDSaturation saturations, PIDParameters parameters, float sampleTime, float tau);
float PIDcompute(PID* pid, float reference, float measured);
float PIDcascade(PID* pid1, PID* pid2, float reference, float measured);

float getProportionalTerm(PID* pid, float error);
float getIntegralTerm(PID* pid, float error);
float getDerivativeTerm(PID* pid, float error);

float getError(float reference, float measured);
float getOutput(PID* pid);

bool _isLastOutputSaturated(PID* pid);
void _updateTerms(PID* pid, float error);
