#include "app_task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_protocol.h"
#include "app_referee.h"
#include "alg_smc.h"
#include "bsp_fdcan.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "app_unit_test.h"
#include "dev_bmi088.h"
#include "dev_dji_dt7.h"
#include "dev_dji_motor.h"
#include "dev_dm_motor.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

// 上赛季Standard机器人的Skywalker版重写。这个文件集中覆盖四个RTOS任务入口，
// 方便后续将整车行为作为模板或范例整体替换。

enum {
  // 1U: 接入裁判系统；0U: 不接入裁判系统，使用本文件内默认参数。
  kStandardRefereeEnabled = 0U,
  // UART1 FireWater遥测；启用裁判系统时自动停用，避免共用串口。
  kStandardVofaEnabled = 1U,
  // 整车功能开关；视觉链路使用USART10。
  kStandardChassisEnabled = 1U,
  kStandardGimbalEnabled = 1U,
  kStandardShootEnabled = 1U,
  kStandardVisionEnabled = 1U,
  // 0U: 摇杆直接给Pitch目标，便于滑模阶跃调参；1U: 使用平滑轨迹。
  kStandardPitchRcTrajectoryEnabled = 0U,
  kStandardKeyboardMouseEnabled = 1U,
  kStandardRemoteSwitchSafeModeEnabled = 0U,
  kStandardUnitTestMask = kUnitTestNone,
  kStandardManagerPeriodMs = 10,
  kStandardGimbalPeriodMs = 2,
  kStandardChassisPeriodMs = 2,
  kStandardShootPeriodMs = 2,
  kStandardMotorCount = 4,
  kStandardCanDataLength = 8,
  kStandardDt7OfflineTimeoutMs = 100,
  kStandardDmEnableRepeatCount = 5,
  kStandardImuGyroZAxis = 2,
  kStandardPitchUpperEncoder = 2459+300,
  kStandardPitchLowerEncoder = 1722-300,
  kStandardPitchCenterEncoder = 2070,
};

static const float kStandardChassisRadiusM = 0.15f;
static const float kStandardWheelRadiusM = 0.01f;
static const float kStandardMaxVxMps = 5.0f;
static const float kStandardMaxVyMps = 5.0f;
static const float kStandardMaxOmegaRadps = 10.0f;
static const float kStandardNoRefereeCurrentScale = 0.5f;
static const float kStandardChassisYawRateFeedbackKp = 0.5f;
static const float kStandardChassisAutoSpinOmegaRadps = 4.0f;
static const float kStandardChassisYawWheelDeadband = 0.05f;
static const float kStandardImuYawRateSign = 1.0f;
static const float kStandardGimbalYawRcSign = 1.0f;
static const float kStandardGimbalYawMaxPositionRad = (float)M_PI;
static const float kStandardGimbalYawVelocityLimitRadps = 18.0f;
static const float kStandardGimbalPitchRcSign = 1.0f;
// 视觉Pitch抬头为正；实机抬头时编码器减小。
static const float kStandardGimbalPitchVisionSign = -1.0f;
static const float kStandardGimbalPitchMinRad =
    (float)(kStandardPitchLowerEncoder - kStandardPitchCenterEncoder) /
    8192.0f * 2.0f * (float)M_PI;
static const float kStandardGimbalPitchMaxRad =
    (float)(kStandardPitchUpperEncoder - kStandardPitchCenterEncoder) /
    8192.0f * 2.0f * (float)M_PI;
static const float kStandardGimbalPitchDeadband = 0.05f;
// Treat 90% of the nominal DT7 range as full stick so calibration tolerance
// cannot prevent the command from reaching either mechanical endpoint.
static const float kStandardGimbalPitchRcFullScale = 0.90f;
static const float kStandardGimbalPitchTargetMaxRateRadps = 6.0f;
static const float kStandardGimbalPitchCenterMaxRateRadps = 8.0f;
static const float kStandardGimbalPitchTargetMaxAccelerationRadps2 = 36.0f;
static const float kStandardGimbalPitchTargetMaxBrakingRadps2 = 54.0f;
static const float kStandardGimbalPitchTrajectoryKp = 16.0f;
// The GM6020 reports speed in integer RPM. A light low-pass plus a two-RPM
// deadband prevents that quantization from repeatedly flipping the SMC output.
static const float kStandardPitchVelocityFilterAlpha = 0.15f;
static const float kStandardPitchVelocityDeadbandRadps =
    2.0f * 2.0f * (float)M_PI / 60.0f;
// SMC current/acceleration scale for the bare motor. This is an effective
// control gain to be identified on the real mechanism, not a rotor inertia
// inferred from the motor's electrical or mechanical time constants.
static const float kStandardPitchBareMotorModelGain = 5.0e-4f;
static const float kStandardGm6020TorqueConstantNmPerA = 0.741f;
static const float kStandardGravityAccelerationMps2 = 9.80665f;
static const float kStandardPitchLoadMassKg = 2.0f;
static const float kStandardPitchLoadCenterDistanceM = 0.15f;
static const float kStandardPitchLoadOwnInertiaKgm2 = 0.0f;
// Start conservatively because mass and center distance are only estimates.
// Increase these scales from measured tracking/current data, not in one jump.
static const float kStandardPitchLoadInertiaScale = 0.02f;
static const float kStandardPitchGravityCompensationScale = 0.15f;
// The load falls toward increasing encoder values, so holding current is
// negative at the center angle. Center is assumed to be the horizontal pose.
static const float kStandardPitchGravityCurrentSign = -1.0f;
static const float kStandardMouseYawRadPerCount = 0.0015f;
static const float kStandardImuHeaterTargetTemperatureC = 40.0f;
static const float kStandardPitchOffsetRad =
    (float)kStandardPitchCenterEncoder / 8192.0f * 2.0f * (float)M_PI;
static const float kStandardBulletFeedOmegaRadps = 910.0f;
static const float kStandardFrictionOmegaRadps = 800.0f;
static const float kStandardDefaultChassisPowerW = 100.0f;

// M3508底盘功率估计参数，保留旧车测试得到的基础模型。
static const float kStandardPowerK0 = 0.30f;
static const float kStandardPowerK1 = 0.37f;
static const float kStandardPowerK2 = 8.30f;
static const float kStandardPowerK3 = 0.0f;

static PidConfig g_standard_bmi088_heater_pid = {
    .kp = 0.20f,
    .ki = 0.03f,
    .kd = 0.0f,
    .sampling_time = 0.002f,
    .is_antiwindup_enabled = true,
    .output_max = 1.0f,
    .integral_max = 10.0f,
};

typedef struct {
  float vx;
  float vy;
  float omega_z;
} StandardVelocity;

typedef struct {
  StandardVelocity desired_velocity;
  StandardVelocity actual_velocity;
  float estimated_power;
  float yaw_angle;
  float target_yaw_angle;
  float yaw_rate;
  float control_dt;
  Bmi088Status imu_status;
  bool imu_ready;
} StandardChassisState;

typedef struct {
  float yaw_angle;
  float target_yaw_angle;
  float pitch_angle;
  float yaw_rate;
  float pitch_rate;
} StandardGimbalState;

typedef struct {
  DjiMotor chassis_motor[kStandardMotorCount];
  DjiMotor friction_motor[2];
  DjiMotor bullet_feed_motor;
  DjiMotor pitch_motor;
  SmcEismcObject pitch_smc;
  float pitch_velocity_feedback;
  float pitch_tracking_error_deg;
  float pitch_peak_tracking_error_deg;
  float pitch_gravity_feedforward_current;
  bool pitch_current_saturated;
  bool pitch_velocity_feedback_initialized;
  bool pitch_was_centering;
  bool vision_control_active;
  bool gimbal_safe_mode_active;
  uint8_t yaw_enable_repeat_count;
  DmMotor yaw_motor;
  StandardChassisState chassis;
  StandardGimbalState gimbal;
  uint16_t bullet_count;
  uint32_t chassis_imu_last_tick;
  uint32_t gimbal_last_tick;
  uint32_t keyboard_mouse_last_frame_count;
  volatile bool is_initialized;
  volatile bool is_initializing;
} StandardApp;

static StandardApp g_standard;

// FireWater纯数字CSV：Pitch目标/实际位置（度）、输出/反馈电流（A）。
static uint16_t StandardFormatVofaTelemetry(char *buffer, size_t capacity) {
  if (buffer == NULL || capacity == 0U) {
    return 0U;
  }

  // 一致快照；控制目标和CAN反馈统一为相对中心、抬头为正的rad。
  uint32_t lock = __get_PRIMASK();
  __disable_irq();
  float pitch_target =
      g_standard.is_initialized
          ? kStandardGimbalPitchVisionSign *
                (g_standard.gimbal.pitch_angle - kStandardPitchOffsetRad)
          : 0.0f;
  float pitch_actual =
      g_standard.pitch_motor.state.is_feedback_initialized
          ? kStandardGimbalPitchVisionSign *
                (g_standard.pitch_motor.state.angle - kStandardPitchOffsetRad)
          : 0.0f;
  float pitch_output_current = g_standard.pitch_motor.state.given_current;
  float pitch_feedback_current =
      g_standard.pitch_motor.state.is_feedback_initialized
          ? g_standard.pitch_motor.state.current
          : 0.0f;
  __set_PRIMASK(lock);
  pitch_target *= 180.0f / (float)M_PI;
  pitch_actual *= 180.0f / (float)M_PI;
  int length = snprintf(buffer, capacity, "%.6f,%.6f,%.4f,%.4f\n",
                        (double)pitch_target, (double)pitch_actual,
                        (double)pitch_output_current,
                        (double)pitch_feedback_current);
  if (length < 0 || (size_t)length >= capacity) {
    return 0U;
  }
  return (uint16_t)length;
}

static void StandardSendVofaTelemetry(void) {
  if (kStandardVofaEnabled == 0U || kStandardRefereeEnabled != 0U ||
      huart1.gState != HAL_UART_STATE_READY) {
    return;
  }
  static char tx_data[kUartBufferSize];
  uint16_t length = StandardFormatVofaTelemetry(tx_data, sizeof(tx_data));
  if (length != 0U) {
    UartTransmit(&huart1, (uint8_t *)tx_data, length);
  }
}

static bool StandardIsUnitTestMode(void) {
  return (uint32_t)kStandardUnitTestMask != (uint32_t)kUnitTestNone;
}

static float StandardClamp(float value, float min, float max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

static float StandardRcNormalize(int16_t value) {
  return StandardClamp((float)value / 660.0f, -1.0f, 1.0f);
}

static float StandardApplyDeadband(float value, float deadband) {
  if (value > -deadband && value < deadband) {
    return 0.0f;
  }
  return value;
}

static float StandardApplyContinuousDeadband(float value, float deadband) {
  if (value > deadband) {
    return value - deadband;
  }
  if (value < -deadband) {
    return value + deadband;
  }
  return 0.0f;
}

static bool StandardAutoSpinIsEnabled(void) {
  return SwitchIsUp(g_dt7_object.rocker.sw1);
}

static float StandardWrapPi(float angle) {
  while (angle > (float)M_PI) {
    angle -= 2.0f * (float)M_PI;
  }
  while (angle < -(float)M_PI) {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

static void StandardUpdateKeyboardMouseInput(void) {
  if (kStandardKeyboardMouseEnabled == 0U) {
    g_standard.keyboard_mouse_last_frame_count = Dt7FrameCount();
    return;
  }

  uint32_t frame_count = Dt7FrameCount();
  if (frame_count == g_standard.keyboard_mouse_last_frame_count) {
    return;
  }
  g_standard.keyboard_mouse_last_frame_count = frame_count;

  g_standard.gimbal.target_yaw_angle = StandardWrapPi(
      g_standard.gimbal.target_yaw_angle +
      (float)g_dt7_object.mouse.x * kStandardMouseYawRadPerCount);
}

static float StandardChassisYawRateCommand(void) {
  float wheel_input = StandardApplyDeadband(
      StandardRcNormalize(g_dt7_object.rocker.wheel),
      kStandardChassisYawWheelDeadband);
  float yaw_input = -wheel_input - (float)g_dt7_object.key.q +
                    (float)g_dt7_object.key.e;
  float yaw_rate =
      StandardClamp(yaw_input, -1.0f, 1.0f) * kStandardMaxOmegaRadps;
  return StandardClamp(yaw_rate, -kStandardMaxOmegaRadps,
                       kStandardMaxOmegaRadps);
}

static float StandardGimbalYawAngleCommand(void) {
  float yaw_input = StandardRcNormalize(g_dt7_object.rocker.ch0);
  return kStandardGimbalYawRcSign * yaw_input *
         kStandardGimbalYawMaxPositionRad;
}

static float StandardGimbalPitchAngleCommand(void) {
  float pitch_input = StandardRcNormalize(g_dt7_object.rocker.ch1);
  if (pitch_input > kStandardGimbalPitchDeadband) {
    pitch_input =
        (pitch_input - kStandardGimbalPitchDeadband) /
        (kStandardGimbalPitchRcFullScale - kStandardGimbalPitchDeadband);
  } else if (pitch_input < -kStandardGimbalPitchDeadband) {
    pitch_input =
        (pitch_input + kStandardGimbalPitchDeadband) /
        (kStandardGimbalPitchRcFullScale - kStandardGimbalPitchDeadband);
  } else {
    pitch_input = 0.0f;
  }
  pitch_input = StandardClamp(pitch_input, -1.0f, 1.0f);
  float pitch_offset = kStandardGimbalPitchRcSign * pitch_input;
  float pitch_range = pitch_offset >= 0.0f
                          ? kStandardGimbalPitchMaxRad
                          : -kStandardGimbalPitchMinRad;
  return kStandardPitchOffsetRad + pitch_offset * pitch_range;
}

static void StandardPackDjiCurrents(const DjiMotor *motor_0,
                                    const DjiMotor *motor_1,
                                    const DjiMotor *motor_2,
                                    const DjiMotor *motor_3,
                                    uint8_t tx_data[8]) {
  const DjiMotor *motors[4] = {motor_0, motor_1, motor_2, motor_3};
  memset(tx_data, 0, 8);
  for (uint8_t i = 0; i < 4U; i++) {
    if (motors[i] == NULL) {
      continue;
    }
    int16_t current_units = DjiMotorCurrentAmperesToUnits(
        motors[i]->type, motors[i]->state.given_current);
    tx_data[i * 2U] = (uint8_t)(current_units >> 8);
    tx_data[i * 2U + 1U] = (uint8_t)(current_units & 0xFFU);
  }
}

static void StandardTransmitYawMotorCommand(DmMotorCommand command) {
  uint8_t tx_data[kStandardCanDataLength] = {0};
  DmMotorPackCommand(&g_standard.yaw_motor, command, tx_data);
  FdcanTransmit(&hfdcan2, tx_data, kStandardCanDataLength,
                DmMotorCommandTxId(&g_standard.yaw_motor));
}

static void StandardFdcan2RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  switch (std_id) {
    case 0x201U:
      DjiMotorUpdate(&g_standard.friction_motor[0], rx_buffer);
      break;
    case 0x202U:
      DjiMotorUpdate(&g_standard.friction_motor[1], rx_buffer);
      break;
    case 0x20BU:
      DjiMotorUpdate(&g_standard.pitch_motor, rx_buffer);
      break;
    default:
      if (std_id == g_standard.yaw_motor.state.id ||
          std_id == g_standard.yaw_motor.state.master_id) {
        DmMotorUpdate(&g_standard.yaw_motor, rx_buffer);
      }
      break;
  }
}

static void StandardFdcan3RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  if (std_id >= 0x201U && std_id <= 0x204U) {
    DjiMotorUpdate(&g_standard.chassis_motor[std_id - 0x201U], rx_buffer);
  } else if (std_id == 0x205U) {
    DjiMotorUpdate(&g_standard.bullet_feed_motor, rx_buffer);
  }
}

static void StandardMotorInit(void) {
  const float pitch_load_inertia =
      kStandardPitchLoadOwnInertiaKgm2 +
      kStandardPitchLoadMassKg * kStandardPitchLoadCenterDistanceM *
          kStandardPitchLoadCenterDistanceM;
  const float pitch_model_gain =
      kStandardPitchBareMotorModelGain +
      kStandardPitchLoadInertiaScale * pitch_load_inertia /
          kStandardGm6020TorqueConstantNmPerA;
  const SmcEismcConfig gm6020_pitch_smc_config = {
      .sampling_time = 0.002f,
      .model_gain = pitch_model_gain,
      .reaching_gain = 30.0f,
      // A higher c1 reduces ramp-following error. c2 is kept well below c1,
      // but is non-zero so that friction/gravity cannot leave a static offset.
      .position_gain = 44.0f,
      .integral_gain = 27.0f,
      .switching_gain = 2.0f,
      .boundary_layer = 0.9f,
      .output_limit = 3.0f,
      .integral_limit = 2.0f,
      // One encoder count is about 0.044 degree, below the 0.1-degree target.
      .position_deadband = 1.0f * 2.0f * (float)M_PI / 8192.0f,
  };
  static PidConfig m2006_speed_pid = {
      .kp = 1.515502929687f,
      .ki = 1.165771484375f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 5.0f,
      .integral_max = 6.103515625f,
  };
  static PidConfig m3508_speed_pid = {
      .kp = 0.0515502929687f,
      .ki = 0.01f,
      .kd = 0.02f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 14.6484375f,
      .integral_max = 6.103515625f,
      .is_deadband_enabled = true,
      .deadband_threshold = 1.0f,
  };
  static PidConfig gm6020_speed_pid = {
      .kp = 64.00f,
      .ki = 100.0f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 0.9f,
      .integral_max = 0.9f,
  };
  static PidConfig gm6020_position_pid = {
      .kp = 600.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_feedforward_enabled = false,
      .kff = 0.0f,
      .is_antiwindup_enabled = true,
      .output_max = 10.0f,
      .integral_max = 10.0f,
  };
  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    DjiMotorInit(&g_standard.chassis_motor[i], kDjiMotorM3508,
                 kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  }
  DjiMotorInit(&g_standard.friction_motor[0], kDjiMotorM3508,
               kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  DjiMotorInit(&g_standard.friction_motor[1], kDjiMotorM3508,
               kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  DjiMotorInit(&g_standard.bullet_feed_motor, kDjiMotorM2006,
               kDjiMotorModeSpeed, &m2006_speed_pid, NULL);
  DjiMotorInit(&g_standard.pitch_motor, kDjiMotorGm6020,
               kDjiMotorModePosition, &gm6020_speed_pid,
               &gm6020_position_pid);
  SmcEismcInit(&g_standard.pitch_smc, &gm6020_pitch_smc_config);
  DmMotorInit(&g_standard.yaw_motor, kDmMotorTypeJ4310,
              kDmMotorModePositionVelocity, 0x06U, 0x11U);
}

static void StandardSharedInit(void) {
  if (g_standard.is_initialized) {
    return;
  }

  uint32_t lock = osKernelLock();
  if (g_standard.is_initialized || g_standard.is_initializing) {
    (void)osKernelRestoreLock(lock);
    while (!g_standard.is_initialized) {
      osDelay(1);
    }
    return;
  }
  memset(&g_standard, 0, sizeof(g_standard));
  g_standard.is_initializing = true;
  (void)osKernelRestoreLock(lock);

  StandardMotorInit();
  if (kStandardGimbalEnabled != 0U || kStandardShootEnabled != 0U) {
    FdcanInit(&hfdcan2, StandardFdcan2RxCallback);
  }
  if (kStandardChassisEnabled != 0U || kStandardShootEnabled != 0U) {
    FdcanInit(&hfdcan3, StandardFdcan3RxCallback);
  }
  UartInit(&huart5, Dt7RxCallback);
  if (kStandardVisionEnabled != 0U) {
    VisionProtocolInit();
  }
  if (kStandardRefereeEnabled != 0U) {
    RefereeInit(&g_referee, &huart1, &huart1);
  } else if (kStandardVofaEnabled != 0U) {
    UartInit(&huart1, NULL);
  }
  if (kStandardChassisEnabled != 0U) {
    g_standard.chassis.imu_status = Bmi088Init(&g_bmi088, &hspi2);
    if (g_standard.chassis.imu_status == kBmi088NoError) {
      Bmi088HeaterInit(&g_bmi088, &htim3, TIM_CHANNEL_4,
                       &g_standard_bmi088_heater_pid,
                       kStandardImuHeaterTargetTemperatureC);
      g_standard.chassis.imu_status =
          Bmi088CalibrateGyroOffset(&g_bmi088, 300U, 1U);
      g_standard.chassis.imu_ready =
          g_standard.chassis.imu_status == kBmi088NoError;
    }
    g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
    g_standard.gimbal.target_yaw_angle = g_standard.chassis.target_yaw_angle;
  }
  g_standard.gimbal.pitch_angle = kStandardPitchOffsetRad;

  // Wait until the remote-control link becomes active before enabling the DM
  // motor. The motor may not be ready when this shared initialization runs.
  g_standard.gimbal_safe_mode_active = true;
  g_standard.chassis_imu_last_tick = osKernelGetTickCount();
  g_standard.gimbal_last_tick = osKernelGetTickCount();
  g_standard.is_initialized = true;
  g_standard.is_initializing = false;
}

static float StandardEstimateChassisPower(void) {
  float sum_power = 0.0f;
  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    float current = g_standard.chassis_motor[i].state.given_current;
    float omega = g_standard.chassis_motor[i].state.omega;
    float torque = kStandardPowerK0 * current;
    float wheel_power = torque * omega + kStandardPowerK1 * fabsf(omega) +
                        kStandardPowerK2 * torque * torque +
                        kStandardPowerK3 / (float)kStandardMotorCount;
    if (wheel_power > 0.0f) {
      sum_power += wheel_power;
    }
  }
  return sum_power;
}

static void StandardApplyChassisPowerLimit(float max_power) {
  float command_power = StandardEstimateChassisPower();
  if (max_power <= 0.0f) {
    for (uint8_t i = 0; i < kStandardMotorCount; i++) {
      g_standard.chassis_motor[i].state.given_current = 0.0f;
    }
    g_standard.chassis.estimated_power = 0.0f;
    return;
  }

  if (command_power > max_power) {
    float scale = max_power / command_power;
    for (uint8_t i = 0; i < kStandardMotorCount; i++) {
      g_standard.chassis_motor[i].state.given_current *= scale;
    }
    g_standard.chassis.estimated_power = max_power;
  } else {
    g_standard.chassis.estimated_power = command_power;
  }
}

static float StandardChassisPowerLimit(void) {
  if (kStandardRefereeEnabled != 0U &&
      g_referee.game_robot_state.chassis_power_limit > 0U) {
    return (float)g_referee.game_robot_state.chassis_power_limit;
  }
  return kStandardDefaultChassisPowerW;
}

static void StandardChassisImuUpdate(void) {
  uint32_t now = osKernelGetTickCount();
  uint32_t elapsed_ms = now - g_standard.chassis_imu_last_tick;
  g_standard.chassis_imu_last_tick = now;
  g_standard.chassis.control_dt = (float)elapsed_ms * 0.001f;

  if (!g_standard.chassis.imu_ready) {
    return;
  }

  Bmi088Status status = Bmi088Update(&g_bmi088);
  if (g_bmi088.is_heater_enabled) {
    (void)Bmi088HeaterControl(&g_bmi088);
  }
  g_standard.chassis.imu_status = status;
  if (status != kBmi088NoError) {
    g_standard.chassis.imu_ready = false;
    g_standard.chassis.yaw_rate = 0.0f;
    g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
    return;
  }

  float yaw_rate =
      kStandardImuYawRateSign * g_bmi088.corrected_gyro[kStandardImuGyroZAxis];
  g_standard.chassis.yaw_rate = yaw_rate;
  g_standard.chassis.yaw_angle = StandardWrapPi(
      g_standard.chassis.yaw_angle + yaw_rate * (float)elapsed_ms * 0.001f);
}

static void StandardChassisKinematics(void) {
  float vx_input = -StandardRcNormalize(g_dt7_object.rocker.ch3) -
                   (float)g_dt7_object.key.w + (float)g_dt7_object.key.s;
  float vy_input = StandardRcNormalize(g_dt7_object.rocker.ch2) +
                   (float)g_dt7_object.key.a - (float)g_dt7_object.key.d;
  float desired_yaw_rate = StandardChassisYawRateCommand();
  bool auto_spin_enabled = StandardAutoSpinIsEnabled();
  if (auto_spin_enabled) {
    if (g_standard.chassis.imu_ready) {
      g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
    }
    desired_yaw_rate += kStandardChassisAutoSpinOmegaRadps;
    desired_yaw_rate = StandardClamp(desired_yaw_rate, -kStandardMaxOmegaRadps,
                                     kStandardMaxOmegaRadps);
  }

  g_standard.chassis.desired_velocity.vx =
      StandardClamp(vx_input, -1.0f, 1.0f) * kStandardMaxVxMps;
  g_standard.chassis.desired_velocity.vy =
      StandardClamp(vy_input, -1.0f, 1.0f) * kStandardMaxVyMps;
  g_standard.chassis.desired_velocity.omega_z = desired_yaw_rate;

  float field_vx = g_standard.chassis.desired_velocity.vx;
  float field_vy = g_standard.chassis.desired_velocity.vy;
  float yaw = g_standard.chassis.imu_ready ? g_standard.chassis.yaw_angle : 0.0f;
  float cos_yaw = cosf(yaw);
  float sin_yaw = sinf(yaw);
  float vx = cos_yaw * field_vx + sin_yaw * field_vy;
  float vy = -sin_yaw * field_vx + cos_yaw * field_vy;
  float wz = desired_yaw_rate;
  if (g_standard.chassis.imu_ready) {
    wz += kStandardChassisYawRateFeedbackKp *
          (desired_yaw_rate - g_standard.chassis.yaw_rate);
  }
  wz = StandardClamp(wz, -kStandardMaxOmegaRadps, kStandardMaxOmegaRadps);
  g_standard.chassis.actual_velocity.vx = vx;
  g_standard.chassis.actual_velocity.vy = vy;
  g_standard.chassis.actual_velocity.omega_z = g_standard.chassis.yaw_rate;
  float rotation_speed = wz * kStandardChassisRadiusM;

  g_standard.chassis_motor[0].state.given_omega =
      (-0.707f * vx + 0.707f * vy + rotation_speed) /
      kStandardWheelRadiusM;
  g_standard.chassis_motor[1].state.given_omega =
      (-0.707f * vx - 0.707f * vy + rotation_speed) /
      kStandardWheelRadiusM;
  g_standard.chassis_motor[2].state.given_omega =
      (0.707f * vx - 0.707f * vy + rotation_speed) /
      kStandardWheelRadiusM;
  g_standard.chassis_motor[3].state.given_omega =
      (0.707f * vx + 0.707f * vy + rotation_speed) / kStandardWheelRadiusM;
}

static void StandardChassisControl(void) {
  StandardChassisImuUpdate();
  StandardChassisKinematics();

  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    DjiMotorCurrentCalculate(&g_standard.chassis_motor[i]);
  }
  if (kStandardRefereeEnabled != 0U) {
    StandardApplyChassisPowerLimit(StandardChassisPowerLimit());
  } else {
    for (uint8_t i = 0; i < kStandardMotorCount; i++) {
      g_standard.chassis_motor[i].state.given_current *=
          kStandardNoRefereeCurrentScale;
    }
    g_standard.chassis.estimated_power = StandardEstimateChassisPower();
  }
  uint8_t tx_data[kStandardCanDataLength] = {0};
  StandardPackDjiCurrents(&g_standard.chassis_motor[0],
                          &g_standard.chassis_motor[1],
                          &g_standard.chassis_motor[2],
                          &g_standard.chassis_motor[3], tx_data);
  FdcanTransmit(&hfdcan3, tx_data, kStandardCanDataLength, 0x200U);
}

static void StandardGimbalControl(void) {
  uint32_t now = osKernelGetTickCount();
  uint32_t elapsed_ticks = now - g_standard.gimbal_last_tick;
  g_standard.gimbal_last_tick = now;
  // 每次控制都更新时间，避免从视觉切回遥控时积累一个很大的dt。
  float dt = StandardClamp((float)elapsed_ticks /
                               (float)osKernelGetTickFreq(),
                           0.0f, 0.02f);
  VisionToGimbal command;
  bool vision_active = kStandardVisionEnabled != 0U &&
                       VisionProtocolGetCommand(&command);
  if (vision_active != g_standard.vision_control_active) {
    SmcEismcResetIntegral(&g_standard.pitch_smc);
  }
  g_standard.vision_control_active = vision_active;
  if (vision_active) {
    g_standard.pitch_was_centering = false;
    g_standard.gimbal.target_yaw_angle = StandardClamp(
        command.yaw, -kStandardGimbalYawMaxPositionRad,
        kStandardGimbalYawMaxPositionRad);
    // DM位置速度模式的速度字段是运动速度上限，保持已有的18 rad/s。
    g_standard.gimbal.yaw_rate = command.yaw_velocity;
    float motor_pitch_offset = kStandardGimbalPitchVisionSign * command.pitch;
    g_standard.gimbal.pitch_angle =
        motor_pitch_offset + kStandardPitchOffsetRad;
    g_standard.gimbal.pitch_rate = StandardClamp(
        kStandardGimbalPitchVisionSign * command.pitch_velocity,
        -kStandardGimbalPitchTargetMaxRateRadps,
        kStandardGimbalPitchTargetMaxRateRadps);
    if ((motor_pitch_offset <= kStandardGimbalPitchMinRad &&
         g_standard.gimbal.pitch_rate < 0.0f) ||
        (motor_pitch_offset >= kStandardGimbalPitchMaxRad &&
         g_standard.gimbal.pitch_rate > 0.0f)) {
      g_standard.gimbal.pitch_rate = 0.0f;
    }
  } else {
    // Map the full DT7 ch0 range directly to the DM motor position range used
    // by the mechanism: -660 -> -pi rad, 0 -> 0 rad, 660 -> pi rad.
    g_standard.gimbal.target_yaw_angle = StandardGimbalYawAngleCommand();
    g_standard.gimbal.yaw_rate = 0.0f;

    float requested_pitch_angle = StandardGimbalPitchAngleCommand();
    bool is_centering =
        fabsf(requested_pitch_angle - kStandardPitchOffsetRad) < 1.0e-6f;
    if (is_centering && !g_standard.pitch_was_centering) {
      // Clear the integral accumulated at the previous stick position. This
      // is the same purpose as the open-source Integval_Clear() interface.
      SmcEismcResetIntegral(&g_standard.pitch_smc);
    }
    g_standard.pitch_was_centering = is_centering;
    if (kStandardPitchRcTrajectoryEnabled == 0U) {
      g_standard.gimbal.pitch_angle = requested_pitch_angle;
      g_standard.gimbal.pitch_rate = 0.0f;
    } else {
      // Proportional deceleration and acceleration limits smooth the target.
      float pitch_target_delta =
          requested_pitch_angle - g_standard.gimbal.pitch_angle;
      float pitch_target_max_rate =
          is_centering ? kStandardGimbalPitchCenterMaxRateRadps
                       : kStandardGimbalPitchTargetMaxRateRadps;
      float desired_pitch_rate = StandardClamp(
          kStandardGimbalPitchTrajectoryKp * pitch_target_delta,
          -pitch_target_max_rate, pitch_target_max_rate);
      bool is_pitch_braking =
          (g_standard.gimbal.pitch_rate > 0.0f &&
           desired_pitch_rate < g_standard.gimbal.pitch_rate) ||
          (g_standard.gimbal.pitch_rate < 0.0f &&
           desired_pitch_rate > g_standard.gimbal.pitch_rate);
      float pitch_rate_max_change =
          (is_pitch_braking
               ? kStandardGimbalPitchTargetMaxBrakingRadps2
               : kStandardGimbalPitchTargetMaxAccelerationRadps2) *
          dt;
      g_standard.gimbal.pitch_rate += StandardClamp(
          desired_pitch_rate - g_standard.gimbal.pitch_rate,
          -pitch_rate_max_change, pitch_rate_max_change);

      float pitch_target_step = g_standard.gimbal.pitch_rate * dt;
      if ((pitch_target_delta >= 0.0f &&
           pitch_target_step >= pitch_target_delta) ||
          (pitch_target_delta <= 0.0f &&
           pitch_target_step <= pitch_target_delta)) {
        g_standard.gimbal.pitch_angle = requested_pitch_angle;
        g_standard.gimbal.pitch_rate = 0.0f;
      } else {
        g_standard.gimbal.pitch_angle += pitch_target_step;
      }
    }
  }

  g_standard.gimbal.pitch_angle =
      StandardClamp(g_standard.gimbal.pitch_angle,
                    kStandardPitchOffsetRad + kStandardGimbalPitchMinRad,
                    kStandardPitchOffsetRad + kStandardGimbalPitchMaxRad);
  g_standard.pitch_motor.state.given_angle = g_standard.gimbal.pitch_angle;
  if (g_standard.pitch_motor.state.is_feedback_initialized) {
    float pitch_position_error = g_standard.pitch_motor.state.angle -
                                 g_standard.pitch_motor.state.given_angle;
    g_standard.pitch_tracking_error_deg =
        pitch_position_error * 180.0f / (float)M_PI;
    float pitch_absolute_error_deg =
        fabsf(g_standard.pitch_tracking_error_deg);
    if (pitch_absolute_error_deg > g_standard.pitch_peak_tracking_error_deg) {
      g_standard.pitch_peak_tracking_error_deg = pitch_absolute_error_deg;
    }
    float pitch_raw_velocity = g_standard.pitch_motor.state.raw_omega;
    if (!g_standard.pitch_velocity_feedback_initialized) {
      g_standard.pitch_velocity_feedback = pitch_raw_velocity;
      g_standard.pitch_velocity_feedback_initialized = true;
    } else {
      g_standard.pitch_velocity_feedback +=
          kStandardPitchVelocityFilterAlpha *
          (pitch_raw_velocity - g_standard.pitch_velocity_feedback);
    }
    float pitch_velocity_error = StandardApplyContinuousDeadband(
        g_standard.pitch_velocity_feedback - g_standard.gimbal.pitch_rate,
        kStandardPitchVelocityDeadbandRadps);
    float pitch_smc_current = SmcEismcCalculate(
        &g_standard.pitch_smc, pitch_position_error, pitch_velocity_error);
    float pitch_relative_angle =
        g_standard.pitch_motor.state.angle - kStandardPitchOffsetRad;
    g_standard.pitch_gravity_feedforward_current =
        kStandardPitchGravityCurrentSign * kStandardPitchLoadMassKg *
        kStandardGravityAccelerationMps2 *
        kStandardPitchLoadCenterDistanceM * cosf(pitch_relative_angle) *
        kStandardPitchGravityCompensationScale /
        kStandardGm6020TorqueConstantNmPerA;
    float pitch_unclamped_current =
        pitch_smc_current + g_standard.pitch_gravity_feedforward_current;
    g_standard.pitch_motor.state.given_current = StandardClamp(
        pitch_unclamped_current, -kDjiGm6020MotorCurrentLimitA,
        kDjiGm6020MotorCurrentLimitA);
    g_standard.pitch_current_saturated =
        fabsf(pitch_unclamped_current) > kDjiGm6020MotorCurrentLimitA;
  } else {
    g_standard.pitch_motor.state.given_current = 0.0f;
    g_standard.pitch_velocity_feedback = 0.0f;
    g_standard.pitch_tracking_error_deg = 0.0f;
    g_standard.pitch_peak_tracking_error_deg = 0.0f;
    g_standard.pitch_gravity_feedforward_current = 0.0f;
    g_standard.pitch_current_saturated = false;
    g_standard.pitch_velocity_feedback_initialized = false;
    SmcEismcReset(&g_standard.pitch_smc);
  }

  // GM6020 pitch 在 FDCAN2，反馈 ID 0x20B 为电机 ID 7，对应 0x2FE 第3路电流。
  uint8_t pitch_tx_data[kStandardCanDataLength] = {0};
  StandardPackDjiCurrents(NULL, NULL, &g_standard.pitch_motor, NULL,
                          pitch_tx_data);
  FdcanTransmit(&hfdcan2, pitch_tx_data, kStandardCanDataLength,
                0x2FEU);

  if (g_standard.yaw_motor.state.error_code != 0U) {
    StandardTransmitYawMotorCommand(kDmMotorCommandClearError);
    StandardTransmitYawMotorCommand(kDmMotorCommandEnable);
  } else {
    g_standard.yaw_motor.state.given_angle =
        g_standard.gimbal.target_yaw_angle;
    g_standard.yaw_motor.state.given_omega =
        kStandardGimbalYawVelocityLimitRadps;
    uint8_t yaw_tx_data[kStandardCanDataLength] = {0};
    DmMotorPackPositionVelocity(&g_standard.yaw_motor, yaw_tx_data);
    FdcanTransmit(&hfdcan2, yaw_tx_data, kStandardCanDataLength,
                  DmMotorTxId(&g_standard.yaw_motor));
  }
}

static void StandardShootControl(void) {
  VisionToGimbal command;
  bool vision_should_feed = kStandardVisionEnabled != 0U &&
                            VisionProtocolGetCommand(&command) &&
                            command.mode == 2U;
  bool should_feed = SwitchIsUp(g_dt7_object.rocker.sw2) ||
                     g_dt7_object.mouse.l != 0U ||
                     vision_should_feed;
  bool should_spin = SwitchIsUp(g_dt7_object.rocker.sw2) || should_feed;

  g_standard.bullet_feed_motor.state.given_omega =
      should_feed ? kStandardBulletFeedOmegaRadps : 0.0f;
  g_standard.friction_motor[0].state.given_omega =
      should_spin ? -kStandardFrictionOmegaRadps : 0.0f;
  g_standard.friction_motor[1].state.given_omega =
      should_spin ? kStandardFrictionOmegaRadps : 0.0f;

  if (should_feed) {
    DjiMotorCurrentCalculate(&g_standard.bullet_feed_motor);
  } else {
    g_standard.bullet_feed_motor.state.given_current = 0.0f;
  }
  uint8_t bullet_tx_data[kStandardCanDataLength] = {0};
  StandardPackDjiCurrents(&g_standard.bullet_feed_motor, NULL, NULL, NULL,
                          bullet_tx_data);
  FdcanTransmit(&hfdcan3, bullet_tx_data, kStandardCanDataLength, 0x1FFU);

  if (should_spin) {
    DjiMotorCurrentCalculate(&g_standard.friction_motor[0]);
    DjiMotorCurrentCalculate(&g_standard.friction_motor[1]);
  } else {
    g_standard.friction_motor[0].state.given_current = 0.0f;
    g_standard.friction_motor[1].state.given_current = 0.0f;
  }
  uint8_t friction_tx_data[kStandardCanDataLength] = {0};
  StandardPackDjiCurrents(&g_standard.friction_motor[0],
                          &g_standard.friction_motor[1], NULL, NULL,
                          friction_tx_data);
  FdcanTransmit(&hfdcan2, friction_tx_data, kStandardCanDataLength,
                0x200U);
}

static void StandardStopChassisMotors(void) {
  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    g_standard.chassis_motor[i].state.given_omega = 0.0f;
    g_standard.chassis_motor[i].state.given_current = 0.0f;
  }
  g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
  g_standard.chassis_imu_last_tick = osKernelGetTickCount();
}

static void StandardStopGimbalMotors(void) {
  g_standard.gimbal.target_yaw_angle = g_standard.chassis.target_yaw_angle;
  g_standard.gimbal.pitch_angle = kStandardPitchOffsetRad;
  g_standard.gimbal.pitch_rate = 0.0f;
  g_standard.pitch_motor.state.given_angle = g_standard.gimbal.pitch_angle;
  g_standard.pitch_motor.state.given_omega = 0.0f;
  g_standard.pitch_motor.state.given_current = 0.0f;
  g_standard.pitch_velocity_feedback = 0.0f;
  g_standard.pitch_tracking_error_deg = 0.0f;
  g_standard.pitch_peak_tracking_error_deg = 0.0f;
  g_standard.pitch_gravity_feedforward_current = 0.0f;
  g_standard.pitch_current_saturated = false;
  g_standard.pitch_velocity_feedback_initialized = false;
  g_standard.pitch_was_centering = false;
  g_standard.vision_control_active = false;
  PidReset(&g_standard.pitch_motor.state.speed_loop);
  SmcEismcReset(&g_standard.pitch_smc);
  g_standard.yaw_motor.state.given_angle =
      g_standard.yaw_motor.state.angle;
  g_standard.yaw_motor.state.given_omega = 0.0f;
  g_standard.gimbal_last_tick = osKernelGetTickCount();
}

static void StandardStopShootMotors(void) {
  g_standard.bullet_feed_motor.state.given_omega = 0.0f;
  g_standard.bullet_feed_motor.state.given_current = 0.0f;
  g_standard.friction_motor[0].state.given_omega = 0.0f;
  g_standard.friction_motor[0].state.given_current = 0.0f;
  g_standard.friction_motor[1].state.given_omega = 0.0f;
  g_standard.friction_motor[1].state.given_current = 0.0f;
}

static void StandardTransmitGimbalStopCurrents(void) {
  uint8_t pitch_tx_data[kStandardCanDataLength] = {0};
  uint8_t yaw_tx_data[kStandardCanDataLength] = {0};
  DmMotorPackPositionVelocity(&g_standard.yaw_motor, yaw_tx_data);

  FdcanTransmit(&hfdcan2, pitch_tx_data, kStandardCanDataLength,
                0x2FEU);
  FdcanTransmit(&hfdcan2, yaw_tx_data, kStandardCanDataLength,
                DmMotorTxId(&g_standard.yaw_motor));
}

static void StandardTransmitChassisStopCurrents(void) {
  uint8_t tx_data[kStandardCanDataLength] = {0};

  FdcanTransmit(&hfdcan3, tx_data, kStandardCanDataLength, 0x200U);
}

static void StandardTransmitShootStopCurrents(void) {
  uint8_t friction_tx_data[kStandardCanDataLength] = {0};
  uint8_t bullet_tx_data[kStandardCanDataLength] = {0};

  FdcanTransmit(&hfdcan2, friction_tx_data, kStandardCanDataLength,
                0x200U);
  FdcanTransmit(&hfdcan3, bullet_tx_data, kStandardCanDataLength, 0x1FFU);
}

static bool StandardDt7IsOnline(void) {
  if (Dt7FrameCount() == 0U) {
    return false;
  }
  return (uint32_t)(osKernelGetTickCount() - Dt7LastUpdateTick()) <=
         kStandardDt7OfflineTimeoutMs;
}

static bool StandardIsSafeMode(void) {
  if (!StandardDt7IsOnline()) {
    return true;
  }
  return kStandardRemoteSwitchSafeModeEnabled != 0U &&
         SwitchIsDown(g_dt7_object.rocker.sw1);
}

static void StandardEnterGimbalSafeMode(void) {
  StandardStopGimbalMotors();
  StandardTransmitGimbalStopCurrents();
  if (!g_standard.gimbal_safe_mode_active) {
    StandardTransmitYawMotorCommand(kDmMotorCommandDisable);
    g_standard.gimbal_safe_mode_active = true;
  }
  g_standard.yaw_enable_repeat_count = 0U;
}

static void StandardEnterChassisSafeMode(void) {
  StandardStopChassisMotors();
  StandardTransmitChassisStopCurrents();
}

static void StandardEnterShootSafeMode(void) {
  StandardStopShootMotors();
  StandardTransmitShootStopCurrents();
}

static void StandardUpdateVisionTx(void) {
  g_gimbal_to_vision.mode =
      (kStandardGimbalEnabled != 0U && !StandardIsSafeMode()) ? 1U : 0U;
  g_gimbal_to_vision.q[0] = 1.0f;
  g_gimbal_to_vision.q[1] = 0.0f;
  g_gimbal_to_vision.q[2] = 0.0f;
  g_gimbal_to_vision.q[3] = 0.0f;
  g_gimbal_to_vision.yaw = g_standard.yaw_motor.state.angle;
  g_gimbal_to_vision.yaw_velocity = g_standard.yaw_motor.state.omega;
  g_gimbal_to_vision.pitch =
      g_standard.pitch_motor.state.is_feedback_initialized
          ? kStandardGimbalPitchVisionSign *
                (g_standard.pitch_motor.state.angle - kStandardPitchOffsetRad)
          : 0.0f;
  g_gimbal_to_vision.pitch_velocity =
      kStandardGimbalPitchVisionSign * g_standard.pitch_motor.state.omega;
  g_gimbal_to_vision.bullet_speed =
      (kStandardRefereeEnabled != 0U) ? g_referee.shoot_data.bullet_speed
                                      : 0.0f;
  g_gimbal_to_vision.bullet_count = g_standard.bullet_count;
}

uint32_t ManagerTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return UnitTestInit(kStandardUnitTestMask);
  }
  StandardSharedInit();
  return kStandardManagerPeriodMs;
}

void ManagerTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    UnitTestLoop();
    return;
  }
  if (kStandardRefereeEnabled != 0U) {
    RefereeUpdate(&g_referee);
  }
  StandardUpdateKeyboardMouseInput();
  if (kStandardVisionEnabled != 0U) {
    StandardUpdateVisionTx();
    VisionProtocolTransmitGimbalState();
  }
  StandardSendVofaTelemetry();
}

uint32_t GimbalTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return kStandardGimbalPeriodMs;
  }
  if (kStandardGimbalEnabled == 0U) {
    return kStandardGimbalPeriodMs;
  }
  StandardSharedInit();
  return kStandardGimbalPeriodMs;
}

void GimbalTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    return;
  }
  if (kStandardGimbalEnabled == 0U) {
    return;
  }
  if (StandardIsSafeMode()) {
    StandardEnterGimbalSafeMode();
    return;
  }
  if (g_standard.gimbal_safe_mode_active) {
    g_standard.gimbal_safe_mode_active = false;
    g_standard.yaw_enable_repeat_count = kStandardDmEnableRepeatCount;
  }
  if (g_standard.yaw_enable_repeat_count > 0U) {
    StandardTransmitYawMotorCommand(kDmMotorCommandEnable);
    g_standard.yaw_enable_repeat_count--;
  }
  StandardGimbalControl();
}

uint32_t ChassisTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return kStandardChassisPeriodMs;
  }
  if (kStandardChassisEnabled == 0U) {
    return kStandardChassisPeriodMs;
  }
  StandardSharedInit();
  return kStandardChassisPeriodMs;
}

void ChassisTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    return;
  }
  if (kStandardChassisEnabled == 0U) {
    return;
  }
  if (StandardIsSafeMode()) {
    StandardEnterChassisSafeMode();
    return;
  }
  StandardChassisControl();
}

uint32_t ShootTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return kStandardShootPeriodMs;
  }
  if (kStandardShootEnabled == 0U) {
    return kStandardShootPeriodMs;
  }
  StandardSharedInit();
  return kStandardShootPeriodMs;
}

void ShootTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    return;
  }
  if (kStandardShootEnabled == 0U) {
    return;
  }
  if (StandardIsSafeMode()) {
    StandardEnterShootSafeMode();
    return;
  }
  StandardShootControl();
}
