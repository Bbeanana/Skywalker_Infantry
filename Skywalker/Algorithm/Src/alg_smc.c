#include "alg_smc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float SmcClamp(float value, float min, float max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

static float SmcSaturation(float value, float boundary_layer) {
  if (boundary_layer <= 0.0f) {
    if (value > 0.0f) {
      return 1.0f;
    }
    if (value < 0.0f) {
      return -1.0f;
    }
    return 0.0f;
  }
  return SmcClamp(value / boundary_layer, -1.0f, 1.0f);
}

static float SmcEismcOutput(const SmcEismcObject *controller,
                            float position_error, float velocity_error,
                            float position_error_integral) {
  const SmcEismcConfig *config = &controller->config;
  float sliding_surface = config->position_gain * position_error +
                          velocity_error +
                          config->integral_gain * position_error_integral;
  float switching = SmcSaturation(sliding_surface, config->boundary_layer);
  return config->model_gain *
         (-config->position_gain * velocity_error -
          config->integral_gain * position_error -
          config->reaching_gain * sliding_surface -
          config->switching_gain * switching);
}

void SmcEismcInit(SmcEismcObject *controller,
                  const SmcEismcConfig *config) {
  if (controller == NULL || config == NULL) {
    return;
  }

  memset(controller, 0, sizeof(*controller));
  controller->config = *config;
  controller->is_initialized =
      config->sampling_time > 0.0f && config->output_limit > 0.0f &&
      config->integral_limit > 0.0f;
}

void SmcEismcReset(SmcEismcObject *controller) {
  if (controller == NULL) {
    return;
  }
  controller->position_error = 0.0f;
  controller->velocity_error = 0.0f;
  controller->position_error_integral = 0.0f;
  controller->sliding_surface = 0.0f;
  controller->output = 0.0f;
}

void SmcEismcResetIntegral(SmcEismcObject *controller) {
  if (controller == NULL) {
    return;
  }
  controller->position_error_integral = 0.0f;
}

float SmcEismcCalculate(SmcEismcObject *controller, float position_error,
                        float velocity_error) {
  if (controller == NULL || !controller->is_initialized) {
    return 0.0f;
  }

  const SmcEismcConfig *config = &controller->config;
  if (fabsf(position_error) < config->position_deadband) {
    position_error = 0.0f;
  }

  float integral_candidate =
      controller->position_error_integral +
      position_error * config->sampling_time;
  integral_candidate = SmcClamp(integral_candidate, -config->integral_limit,
                                config->integral_limit);
  float unclamped_output = SmcEismcOutput(
      controller, position_error, velocity_error, integral_candidate);

  // Do not integrate further when the position error pushes an already
  // saturated output in the same direction.
  bool blocks_positive_windup =
      unclamped_output > config->output_limit && position_error < 0.0f;
  bool blocks_negative_windup =
      unclamped_output < -config->output_limit && position_error > 0.0f;
  if (!blocks_positive_windup && !blocks_negative_windup) {
    controller->position_error_integral = integral_candidate;
  } else {
    unclamped_output =
        SmcEismcOutput(controller, position_error, velocity_error,
                       controller->position_error_integral);
  }

  controller->position_error = position_error;
  controller->velocity_error = velocity_error;
  controller->sliding_surface =
      config->position_gain * position_error + velocity_error +
      config->integral_gain * controller->position_error_integral;
  controller->output = SmcClamp(unclamped_output, -config->output_limit,
                                config->output_limit);
  return controller->output;
}
