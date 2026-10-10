#ifndef ALG_KALMAN_H
#define ALG_KALMAN_H

#include <stdbool.h>

typedef struct {
  float process_acceleration_variance;
  float position_measurement_variance;
  float velocity_measurement_variance;
  float initial_position_variance;
  float initial_velocity_variance;
} KalmanPositionVelocityConfig;

typedef struct {
  float position;
  float velocity;
  float covariance_00;
  float covariance_01;
  float covariance_10;
  float covariance_11;
  float process_acceleration_variance;
  float position_measurement_variance;
  float velocity_measurement_variance;
  float initial_position_variance;
  float initial_velocity_variance;
  bool is_initialized;
} KalmanPositionVelocityObject;

void KalmanPositionVelocityInit(
    KalmanPositionVelocityObject *filter,
    const KalmanPositionVelocityConfig *config);
void KalmanPositionVelocityReset(KalmanPositionVelocityObject *filter,
                                 float position, float velocity);
void KalmanPositionVelocityUpdate(KalmanPositionVelocityObject *filter,
                                  float position_measurement,
                                  float velocity_measurement, float dt);

#endif  // ALG_KALMAN_H
