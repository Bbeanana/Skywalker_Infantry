#include "alg_kalman.h"

#include <stddef.h>
#include <string.h>

static const float kKalmanMinimumVariance = 1.0e-12f;

static float KalmanPositiveValue(float value, float fallback) {
  return value > 0.0f ? value : fallback;
}

void KalmanPositionVelocityInit(
    KalmanPositionVelocityObject *filter,
    const KalmanPositionVelocityConfig *config) {
  if (filter == NULL || config == NULL) {
    return;
  }

  memset(filter, 0, sizeof(*filter));
  filter->process_acceleration_variance = KalmanPositiveValue(
      config->process_acceleration_variance, kKalmanMinimumVariance);
  filter->position_measurement_variance = KalmanPositiveValue(
      config->position_measurement_variance, kKalmanMinimumVariance);
  filter->velocity_measurement_variance = KalmanPositiveValue(
      config->velocity_measurement_variance, kKalmanMinimumVariance);
  filter->initial_position_variance = KalmanPositiveValue(
      config->initial_position_variance,
      filter->position_measurement_variance);
  filter->initial_velocity_variance = KalmanPositiveValue(
      config->initial_velocity_variance,
      filter->velocity_measurement_variance);
}

void KalmanPositionVelocityReset(KalmanPositionVelocityObject *filter,
                                 float position, float velocity) {
  if (filter == NULL) {
    return;
  }

  filter->position = position;
  filter->velocity = velocity;
  filter->covariance_00 = filter->initial_position_variance;
  filter->covariance_01 = 0.0f;
  filter->covariance_10 = 0.0f;
  filter->covariance_11 = filter->initial_velocity_variance;
  filter->is_initialized = true;
}

void KalmanPositionVelocityUpdate(KalmanPositionVelocityObject *filter,
                                  float position_measurement,
                                  float velocity_measurement, float dt) {
  if (filter == NULL || dt <= 0.0f) {
    return;
  }
  if (!filter->is_initialized) {
    KalmanPositionVelocityReset(filter, position_measurement,
                                velocity_measurement);
    return;
  }

  // Constant-velocity prediction with white acceleration process noise.
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt2 * dt2;
  float process_variance = filter->process_acceleration_variance;

  filter->position += filter->velocity * dt;
  float predicted_00 =
      filter->covariance_00 +
      dt * (filter->covariance_01 + filter->covariance_10) +
      dt2 * filter->covariance_11 + 0.25f * dt4 * process_variance;
  float predicted_01 = filter->covariance_01 +
                       dt * filter->covariance_11 +
                       0.5f * dt3 * process_variance;
  float predicted_10 = filter->covariance_10 +
                       dt * filter->covariance_11 +
                       0.5f * dt3 * process_variance;
  float predicted_11 =
      filter->covariance_11 + dt2 * process_variance;

  // Correct position first (H = [1, 0]).
  float innovation = position_measurement - filter->position;
  float innovation_variance =
      predicted_00 + filter->position_measurement_variance;
  float gain_0 = predicted_00 / innovation_variance;
  float gain_1 = predicted_10 / innovation_variance;
  filter->position += gain_0 * innovation;
  filter->velocity += gain_1 * innovation;

  float corrected_00 = (1.0f - gain_0) * predicted_00;
  float corrected_01 = (1.0f - gain_0) * predicted_01;
  float corrected_10 = predicted_10 - gain_1 * predicted_00;
  float corrected_11 = predicted_11 - gain_1 * predicted_01;
  float corrected_cross = 0.5f * (corrected_01 + corrected_10);
  filter->covariance_00 = corrected_00;
  filter->covariance_01 = corrected_cross;
  filter->covariance_10 = corrected_cross;
  filter->covariance_11 = corrected_11;

  // Correct velocity second (H = [0, 1]).
  innovation = velocity_measurement - filter->velocity;
  innovation_variance =
      filter->covariance_11 + filter->velocity_measurement_variance;
  gain_0 = filter->covariance_01 / innovation_variance;
  gain_1 = filter->covariance_11 / innovation_variance;
  filter->position += gain_0 * innovation;
  filter->velocity += gain_1 * innovation;

  corrected_00 =
      filter->covariance_00 - gain_0 * filter->covariance_10;
  corrected_01 =
      filter->covariance_01 - gain_0 * filter->covariance_11;
  corrected_10 = (1.0f - gain_1) * filter->covariance_10;
  corrected_11 = (1.0f - gain_1) * filter->covariance_11;
  corrected_cross = 0.5f * (corrected_01 + corrected_10);
  filter->covariance_00 = corrected_00;
  filter->covariance_01 = corrected_cross;
  filter->covariance_10 = corrected_cross;
  filter->covariance_11 = corrected_11;
}
