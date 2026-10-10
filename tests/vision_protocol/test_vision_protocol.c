#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "app_protocol.h"
#include "alg_crc.h"
#include "bsp_uart.h"

UART_HandleTypeDef huart10;
static uint32_t tick;
static uint32_t tick_frequency = 1000U;
static uint32_t irq_mask;
static uint8_t transmitted[43];
static UartRxCallback rx_callback;

uint32_t osKernelGetTickCount(void) { return tick; }
uint32_t osKernelGetTickFreq(void) { return tick_frequency; }
uint32_t __get_PRIMASK(void) { return irq_mask; }
void __disable_irq(void) { irq_mask = 1U; }
void __set_PRIMASK(uint32_t mask) { irq_mask = mask; }
void UartInit(UART_HandleTypeDef *uart, UartRxCallback callback) {
  assert(uart == &huart10);
  rx_callback = callback;
}
void UartTransmit(UART_HandleTypeDef *uart, uint8_t *data, uint16_t length) {
  assert(uart == &huart10 && length == sizeof(transmitted));
  memcpy(transmitted, data, length);
}

// Independent bit-wise reference, reflected polynomial 0x8408, no final XOR.
static uint16_t ReferenceCrc(const uint8_t *bytes, size_t length) {
  uint16_t crc = 0xFFFFU;
  for (size_t i = 0; i < length; i++) {
    crc ^= bytes[i];
    for (unsigned bit = 0; bit < 8U; bit++) {
      crc = (uint16_t)((crc >> 1) ^ ((crc & 1U) ? 0x8408U : 0U));
    }
  }
  return crc;
}

static void Seal(uint8_t *frame) {
  uint16_t crc = ReferenceCrc(frame, 27U);
  frame[27] = (uint8_t)crc;
  frame[28] = (uint8_t)(crc >> 8);
}

static void MakeFrame(uint8_t frame[29], uint8_t mode, float yaw) {
  VisionToGimbal command = {
      .head = {'A', 'B'}, .mode = mode, .yaw = yaw,
      .pitch = 0.1f, .pitch_velocity = 0.5f,
  };
  memcpy(frame, &command, sizeof(command));
  Seal(frame);
}

int main(void) {
  VisionToGimbal command;
  VisionProtocolTelemetry telemetry;
  uint8_t frame[29];
  MakeFrame(frame, 1U, 0.75f);
  assert(ReferenceCrc((const uint8_t *)"123456789", 9U) == 0x6F91U);
  assert(GetCrc16(frame, 27U) == ReferenceCrc(frame, 27U));

  // Every possible two-chunk boundary, including splitting the AB header.
  for (uint16_t split = 1U; split < sizeof(frame); split++) {
    VisionProtocolInit();
    assert(rx_callback == VisionProtocolRxCallback);
    assert(!VisionProtocolGetCommand(&command));
    rx_callback(frame, split);
    assert(!VisionProtocolGetCommand(&command));
    VisionProtocolGetTelemetry(&telemetry);
    assert(!telemetry.rx_frame_valid && !telemetry.command_active);
    rx_callback(&frame[split], (uint16_t)(sizeof(frame) - split));
    assert(VisionProtocolGetCommand(&command) && command.yaw == 0.75f);
    VisionProtocolGetTelemetry(&telemetry);
    assert(telemetry.rx_frame_valid && telemetry.command_active);
    assert(memcmp(telemetry.rx_frame, frame, sizeof(frame)) == 0);
    assert(telemetry.command.pitch == 0.1f && telemetry.command.yaw == 0.75f);
  }
  VisionProtocolInit();
  for (uint16_t i = 0U; i < sizeof(frame); i++) { rx_callback(&frame[i], 1U); }
  assert(VisionProtocolGetCommand(&command));

  // Noise, glued frames and a false AB immediately before a valid frame.
  uint8_t stream[90] = {'A', 'B', 'A'};
  memcpy(&stream[3], frame, sizeof(frame));
  MakeFrame(&stream[32], 2U, -0.25f);
  memcpy(&stream[61], frame, sizeof(frame));
  VisionProtocolInit();
  rx_callback(stream, sizeof(stream));
  assert(VisionProtocolGetCommand(&command) && command.yaw == 0.75f);

  // CRC errors cannot refresh an old fire command.
  MakeFrame(frame, 2U, 0.5f);
  tick = 10U;
  rx_callback(frame, sizeof(frame));
  frame[5] ^= 1U;
  tick = 100U;
  rx_callback(frame, sizeof(frame));
  VisionProtocolGetTelemetry(&telemetry);
  assert(!telemetry.rx_frame_valid && telemetry.command_active);
  assert(memcmp(telemetry.rx_frame, frame, sizeof(frame)) == 0);
  assert(telemetry.command.yaw == 0.5f);
  tick = 110U;
  assert(VisionProtocolGetCommand(&command) && command.mode == 2U);
  tick = 111U;
  assert(!VisionProtocolGetCommand(&command));
  VisionProtocolGetTelemetry(&telemetry);
  assert(!telemetry.command_active && telemetry.command.yaw == 0.5f);

  // A valid mode=0 immediately releases control, even after mode=2.
  MakeFrame(frame, 0U, 0.0f);
  rx_callback(frame, sizeof(frame));
  assert(!VisionProtocolGetCommand(&command) && command.mode == 0U);
  VisionProtocolGetTelemetry(&telemetry);
  assert(telemetry.rx_frame_valid && !telemetry.command_active);

  // CRC-valid invalid modes and every NaN/Inf field must be rejected.
  MakeFrame(frame, 3U, 0.0f);
  rx_callback(frame, sizeof(frame));
  assert(!VisionProtocolGetCommand(&command));
  const float invalid_values[] = {NAN, INFINITY, -INFINITY};
  for (size_t value = 0; value < 3U; value++) {
    for (size_t field = 0; field < 6U; field++) {
      MakeFrame(frame, 1U, 0.0f);
      memcpy(&frame[3U + field * sizeof(float)], &invalid_values[value], sizeof(float));
      Seal(frame);
      rx_callback(frame, sizeof(frame));
      assert(!VisionProtocolGetCommand(&command));
    }
  }

  // The timeout works across tick rollover and for non-1kHz tick rates.
  MakeFrame(frame, 1U, 0.0f);
  tick = UINT32_MAX - 49U;
  rx_callback(frame, sizeof(frame));
  tick = 50U;
  assert(VisionProtocolGetCommand(&command));
  tick = 51U;
  assert(!VisionProtocolGetCommand(&command));
  tick_frequency = 100U;
  tick = 0U;
  rx_callback(frame, sizeof(frame));
  tick = 10U;
  assert(VisionProtocolGetCommand(&command));
  tick = 11U;
  assert(!VisionProtocolGetCommand(&command));

  irq_mask = 1U;
  VisionProtocolGetCommand(&command);
  assert(irq_mask == 1U);
  assert(!VisionProtocolGetCommand(NULL));
  rx_callback(NULL, 1U);
  rx_callback(frame, 0U);
  VisionProtocolInit();
  assert(!VisionProtocolGetCommand(&command) && irq_mask == 1U);
  VisionProtocolGetTelemetry(&telemetry);
  assert(!telemetry.rx_frame_valid && !telemetry.command_active);
  assert(telemetry.command.pitch == 0.0f && telemetry.command.yaw == 0.0f);
  for (size_t i = 0U; i < sizeof(telemetry.rx_frame); i++) {
    assert(telemetry.rx_frame[i] == 0U);
  }
  VisionProtocolGetTelemetry(NULL);
  assert(irq_mask == 1U);

  // Outgoing state keeps the existing 43-byte layout and low-byte-first CRC.
  g_gimbal_to_vision.yaw = 0.25f;
  g_gimbal_to_vision.pitch = -0.1f;
  VisionProtocolTransmitGimbalState();
  assert(transmitted[0] == 'A' && transmitted[1] == 'B');
  uint16_t crc = ReferenceCrc(transmitted, 41U);
  assert(transmitted[41] == (uint8_t)crc && transmitted[42] == (uint8_t)(crc >> 8));
  puts("Vision protocol tests passed");
  return 0;
}
