#include "app_protocol.h"

#include <stdbool.h>
#include <math.h>
#include <string.h>

#include "alg_crc.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "usart.h"

GimbalToVision g_gimbal_to_vision = {
    .head = {'A', 'B'},
};

VisionToGimbal g_vision_to_gimbal = {
    .head = {'A', 'B'},
};

_Static_assert(sizeof(GimbalToVision) == 43U, "Vision TX frame size changed");
_Static_assert(sizeof(VisionToGimbal) == 29U, "Vision RX frame size changed");

static uint8_t g_vision_rx_stream[sizeof(VisionToGimbal)];
static uint16_t g_vision_rx_length;
static volatile uint32_t g_vision_last_rx_tick;
static volatile bool g_vision_has_command;
static uint8_t g_vision_last_rx_frame[sizeof(VisionToGimbal)];
static bool g_vision_last_rx_frame_valid;

void VisionProtocolInit(void) {
  uint32_t lock = __get_PRIMASK();
  __disable_irq();
  g_vision_rx_length = 0U;
  g_vision_last_rx_tick = 0U;
  g_vision_has_command = false;
  memset(g_vision_last_rx_frame, 0, sizeof(g_vision_last_rx_frame));
  g_vision_last_rx_frame_valid = false;
  memset(&g_vision_to_gimbal, 0, sizeof(g_vision_to_gimbal));
  g_vision_to_gimbal.head[0] = 'A';
  g_vision_to_gimbal.head[1] = 'B';
  __set_PRIMASK(lock);
  UartInit(&huart10, VisionProtocolRxCallback);
}

static bool VisionProtocolHeaderIsValid(const uint8_t *data) {
  return data != NULL && data[0] == 'A' && data[1] == 'B';
}

static bool VisionProtocolCommandIsValid(const VisionToGimbal *command) {
  return command->mode <= 2U && isfinite(command->yaw) &&
         isfinite(command->yaw_velocity) &&
         isfinite(command->yaw_acceleration) && isfinite(command->pitch) &&
         isfinite(command->pitch_velocity) &&
         isfinite(command->pitch_acceleration);
}

static void VisionProtocolConsumeBytes(uint16_t length) {
  g_vision_rx_length = (uint16_t)(g_vision_rx_length - length);
  memmove(g_vision_rx_stream, &g_vision_rx_stream[length], g_vision_rx_length);
}

void VisionProtocolRxCallback(uint8_t *rx_buffer, uint16_t rx_length) {
  if (rx_buffer == NULL || rx_length == 0U) {
    return;
  }

  // 保留半包；坏帧逐字节重同步，避免跳过紧随其后的有效AB帧。
  for (uint16_t i = 0U; i < rx_length; i++) {
    g_vision_rx_stream[g_vision_rx_length++] = rx_buffer[i];
    while (g_vision_rx_length >= 2U) {
      if (!VisionProtocolHeaderIsValid(g_vision_rx_stream)) {
        VisionProtocolConsumeBytes(1U);
        continue;
      }
      if (g_vision_rx_length < sizeof(VisionToGimbal)) {
        break;
      }
      VisionToGimbal command;
      memcpy(&command, g_vision_rx_stream, sizeof(command));
      memcpy(g_vision_last_rx_frame, g_vision_rx_stream, sizeof(command));
      g_vision_last_rx_frame_valid =
          CheckCrc16(g_vision_rx_stream, sizeof(command)) &&
          VisionProtocolCommandIsValid(&command);
      if (g_vision_last_rx_frame_valid) {
        memcpy(&g_vision_to_gimbal, &command, sizeof(command));
        g_vision_last_rx_tick = osKernelGetTickCount();
        g_vision_has_command = true;
        VisionProtocolConsumeBytes(sizeof(command));
      } else {
        VisionProtocolConsumeBytes(1U);
      }
    }
  }
}

static bool VisionProtocolCommandIsActive(bool has_command,
                                          uint32_t last_rx_tick, uint32_t now,
                                          uint8_t mode) {
  uint32_t tick_frequency = osKernelGetTickFreq();
  return has_command && tick_frequency != 0U &&
         (uint64_t)(uint32_t)(now - last_rx_tick) * 1000U <=
             (uint64_t)kVisionCommandTimeoutMs * tick_frequency &&
         (mode == 1U || mode == 2U);
}

bool VisionProtocolGetCommand(VisionToGimbal *command) {
  if (command == NULL) {
    return false;
  }
  uint32_t lock = __get_PRIMASK();
  __disable_irq();
  bool has_command = g_vision_has_command;
  uint32_t last_rx_tick = g_vision_last_rx_tick;
  uint32_t now = osKernelGetTickCount();
  memcpy(command, &g_vision_to_gimbal, sizeof(*command));
  __set_PRIMASK(lock);

  return VisionProtocolCommandIsActive(has_command, last_rx_tick, now,
                                       command->mode);
}

void VisionProtocolGetTelemetry(VisionProtocolTelemetry *telemetry) {
  if (telemetry == NULL) {
    return;
  }
  uint32_t lock = __get_PRIMASK();
  __disable_irq();
  memcpy(telemetry->rx_frame, g_vision_last_rx_frame,
         sizeof(telemetry->rx_frame));
  telemetry->rx_frame_valid = g_vision_last_rx_frame_valid;
  memcpy(&telemetry->command, &g_vision_to_gimbal, sizeof(telemetry->command));
  bool has_command = g_vision_has_command;
  uint32_t last_rx_tick = g_vision_last_rx_tick;
  uint32_t now = osKernelGetTickCount();
  __set_PRIMASK(lock);
  telemetry->command_active = VisionProtocolCommandIsActive(
      has_command, last_rx_tick, now, telemetry->command.mode);
}

void VisionProtocolTransmitGimbalState(void) {
  g_gimbal_to_vision.head[0] = 'A';
  g_gimbal_to_vision.head[1] = 'B';

  uint8_t tx_buffer[sizeof(GimbalToVision)];
  memcpy(tx_buffer, &g_gimbal_to_vision, sizeof(tx_buffer));
  uint16_t crc16 =
      GetCrc16(tx_buffer, (uint16_t)(sizeof(tx_buffer) - sizeof(uint16_t)));
  tx_buffer[sizeof(tx_buffer) - 2U] = (uint8_t)(crc16 & 0xFFU);
  tx_buffer[sizeof(tx_buffer) - 1U] = (uint8_t)(crc16 >> 8);
  UartTransmit(&huart10, tx_buffer, sizeof(tx_buffer));
}
