#ifndef ALG_SMC_H
#define ALG_SMC_H

#include <stdbool.h>

typedef struct {
  float sampling_time;
  float model_gain;
  float reaching_gain;
  float position_gain;
  float integral_gain;
  float switching_gain;
  float boundary_layer;
  float output_limit;
  float integral_limit;
  float position_deadband;
} SmcEismcConfig;

typedef struct {
  SmcEismcConfig config;
  float position_error;
  float velocity_error;
  float position_error_integral;
  float sliding_surface;
  float output;
  bool is_initialized;
} SmcEismcObject;

void SmcEismcInit(SmcEismcObject *controller,
                  const SmcEismcConfig *config);
void SmcEismcReset(SmcEismcObject *controller);
void SmcEismcResetIntegral(SmcEismcObject *controller);

// Both errors use feedback - reference sign convention.
float SmcEismcCalculate(SmcEismcObject *controller, float position_error,
                        float velocity_error);

#endif  // ALG_SMC_H
