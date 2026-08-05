#include "ftepc_rs485.h"

#include <limits.h>
#include <string.h>

#define FTEPC485_FUNCTION_READ_HOLDING 0x03U
#define FTEPC485_PULSE_START_REGISTER 0x0030U
#define FTEPC485_THREE_PULSE_REGISTERS 0x0006U
#define FTEPC485_READ_THREE_REQUEST_LEN 8U
#define FTEPC485_READ_THREE_RESPONSE_LEN 17U
#define FTEPC485_READ_THREE_BYTE_COUNT 12U
#define FTEPC485_DEFAULT_TIMEOUT_MS 100U

static const int8_t ftepc485_channel_direction[FTEPC485_CHANNEL_COUNT] = {
    -1, -1, -1};

static void Ftepc485_SetTransmit(Ftepc485_Handle *handle)
{
  if ((handle->de_port != NULL) && (handle->de_pin != 0U))
  {
    HAL_GPIO_WritePin(handle->de_port, handle->de_pin, GPIO_PIN_SET);
  }
}

static void Ftepc485_SetReceive(Ftepc485_Handle *handle)
{
  if ((handle->de_port != NULL) && (handle->de_pin != 0U))
  {
    HAL_GPIO_WritePin(handle->de_port, handle->de_pin, GPIO_PIN_RESET);
  }
}

static int32_t Ftepc485_ReadInt32Be(const uint8_t *data)
{
  uint32_t value = ((uint32_t)data[0] << 24) |
                   ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[2] << 8) |
                   (uint32_t)data[3];

  return (int32_t)value;
}

static int32_t Ftepc485_SaturateInt32(int64_t value)
{
  if (value > INT32_MAX)
  {
    return INT32_MAX;
  }
  if (value < INT32_MIN)
  {
    return INT32_MIN;
  }
  return (int32_t)value;
}

static int64_t Ftepc485_DivideRoundNearest(int64_t numerator,
                                           int64_t denominator)
{
  if (denominator <= 0)
  {
    return 0;
  }

  if (numerator >= 0)
  {
    return (numerator + (denominator / 2)) / denominator;
  }

  return -(((-numerator) + (denominator / 2)) / denominator);
}

static int8_t Ftepc485_ChannelDirection(uint8_t channel)
{
  if (channel >= FTEPC485_CHANNEL_COUNT)
  {
    return 0;
  }

  return ftepc485_channel_direction[channel];
}

static void Ftepc485_BuildReadRequest(uint8_t slave_address,
                                      uint16_t start_register,
                                      uint16_t register_count,
                                      uint8_t request[FTEPC485_READ_THREE_REQUEST_LEN])
{
  uint16_t crc;

  request[0] = slave_address;
  request[1] = FTEPC485_FUNCTION_READ_HOLDING;
  request[2] = (uint8_t)(start_register >> 8);
  request[3] = (uint8_t)start_register;
  request[4] = (uint8_t)(register_count >> 8);
  request[5] = (uint8_t)register_count;

  crc = Ftepc485_Crc16(request, 6U);
  request[6] = (uint8_t)crc;
  request[7] = (uint8_t)(crc >> 8);
}

uint16_t Ftepc485_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t index = 0; index < length; index++)
  {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8U; bit++)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

Ftepc485_Status Ftepc485_ParseThreePulseResponse(const uint8_t *response,
                                                 uint16_t length,
                                                 uint8_t slave_address,
                                                 int32_t pulses[3])
{
  uint16_t expected_crc;
  uint16_t frame_crc;

  if ((response == NULL) || (pulses == NULL))
  {
    return FTEPC485_ERROR_ARGUMENT;
  }

  if (length != FTEPC485_READ_THREE_RESPONSE_LEN)
  {
    return FTEPC485_ERROR_FRAME;
  }

  expected_crc = Ftepc485_Crc16(response, (uint16_t)(length - 2U));
  frame_crc = (uint16_t)response[length - 2U] |
              ((uint16_t)response[length - 1U] << 8);
  if (expected_crc != frame_crc)
  {
    return FTEPC485_ERROR_CRC;
  }

  if ((response[0] != slave_address) ||
      (response[1] != FTEPC485_FUNCTION_READ_HOLDING) ||
      (response[2] != FTEPC485_READ_THREE_BYTE_COUNT))
  {
    return FTEPC485_ERROR_FRAME;
  }

  for (uint8_t channel = 0; channel < FTEPC485_CHANNEL_COUNT; channel++)
  {
    pulses[channel] = Ftepc485_ReadInt32Be(&response[3U + (channel * 4U)]);
  }

  return FTEPC485_OK;
}

void Ftepc485_Init(Ftepc485_Handle *handle,
                   UART_HandleTypeDef *uart,
                   GPIO_TypeDef *de_port,
                   uint16_t de_pin,
                   uint8_t slave_address)
{
  if (handle == NULL)
  {
    return;
  }

  memset(handle, 0, sizeof(*handle));
  handle->uart = uart;
  handle->de_port = de_port;
  handle->de_pin = de_pin;
  handle->slave_address = slave_address;
  handle->timeout_ms = FTEPC485_DEFAULT_TIMEOUT_MS;

  if ((de_port != NULL) && (de_pin != 0U))
  {
    Ftepc485_SetReceive(handle);
  }
}

Ftepc485_Status Ftepc485_ReadThreePulses(Ftepc485_Handle *handle,
                                         Ftepc485_Data *data)
{
  uint8_t request[FTEPC485_READ_THREE_REQUEST_LEN];
  uint8_t response[FTEPC485_READ_THREE_RESPONSE_LEN];
  int32_t pulses[FTEPC485_CHANNEL_COUNT];
  Ftepc485_Status status;

  if ((handle == NULL) || (handle->uart == NULL) || (data == NULL))
  {
    return FTEPC485_ERROR_ARGUMENT;
  }

  Ftepc485_BuildReadRequest(handle->slave_address,
                            FTEPC485_PULSE_START_REGISTER,
                            FTEPC485_THREE_PULSE_REGISTERS,
                            request);

  (void)HAL_UART_AbortReceive(handle->uart);
  __HAL_UART_CLEAR_OREFLAG(handle->uart);
  __HAL_UART_CLEAR_FEFLAG(handle->uart);
  __HAL_UART_CLEAR_NEFLAG(handle->uart);

  Ftepc485_SetTransmit(handle);
  if (HAL_UART_Transmit(handle->uart, request, sizeof(request),
                        handle->timeout_ms) != HAL_OK)
  {
    Ftepc485_SetReceive(handle);
    return FTEPC485_ERROR_UART;
  }

  while (__HAL_UART_GET_FLAG(handle->uart, UART_FLAG_TC) == RESET)
  {
  }
  Ftepc485_SetReceive(handle);

  if (HAL_UART_Receive(handle->uart, response, sizeof(response),
                       handle->timeout_ms) != HAL_OK)
  {
    return FTEPC485_ERROR_UART;
  }

  status = Ftepc485_ParseThreePulseResponse(response, sizeof(response),
                                           handle->slave_address, pulses);
  if (status != FTEPC485_OK)
  {
    return status;
  }

  Ftepc485_ConvertPulses(pulses, data);

  return FTEPC485_OK;
}

void Ftepc485_ConvertPulses(const int32_t pulses[3], Ftepc485_Data *data)
{
  if ((pulses == NULL) || (data == NULL))
  {
    return;
  }

  for (uint8_t channel = 0; channel < FTEPC485_CHANNEL_COUNT; channel++)
  {
    int64_t length = (int64_t)pulses[channel] *
                     (int64_t)Ftepc485_ChannelDirection(channel) *
                     (int64_t)FTEPC485_HUNDREDTHS_MM_PER_PULSE_NUM;

    data->pulse[channel] = pulses[channel];
    data->length_hundredths_mm[channel] = Ftepc485_SaturateInt32(
        Ftepc485_DivideRoundNearest(
            length,
            FTEPC485_HUNDREDTHS_MM_PER_PULSE_DEN));
  }
}

int32_t Ftepc485_DeltaToSpeedHundredthsMmS(uint8_t channel,
                                           int32_t delta_pulse,
                                           uint32_t delta_ms)
{
  int64_t speed;

  if (delta_ms == 0U)
  {
    delta_ms = 1U;
  }

  speed = ((int64_t)delta_pulse *
           (int64_t)Ftepc485_ChannelDirection(channel) *
           (int64_t)FTEPC485_HUNDREDTHS_MM_PER_PULSE_NUM * 1000);

  return Ftepc485_SaturateInt32(
      Ftepc485_DivideRoundNearest(
          speed,
          (int64_t)delta_ms * FTEPC485_HUNDREDTHS_MM_PER_PULSE_DEN));
}

Ftepc485_Status Ftepc485_Poll(Ftepc485_Handle *handle,
                              Ftepc485_Data *data,
                              uint32_t now_ms)
{
  Ftepc485_Status status;
  uint32_t delta_ms;

  if ((handle == NULL) || (data == NULL))
  {
    return FTEPC485_ERROR_ARGUMENT;
  }

  status = Ftepc485_ReadThreePulses(handle, data);
  if (status != FTEPC485_OK)
  {
    return status;
  }

  if (handle->has_last_sample == 0U)
  {
    for (uint8_t channel = 0; channel < FTEPC485_CHANNEL_COUNT; channel++)
    {
      data->speed_hundredths_mm_s[channel] = 0;
      handle->last_pulse[channel] = data->pulse[channel];
    }
    handle->last_tick_ms = now_ms;
    handle->has_last_sample = 1U;
    return FTEPC485_OK;
  }

  delta_ms = now_ms - handle->last_tick_ms;
  if (delta_ms == 0U)
  {
    delta_ms = 1U;
  }

  for (uint8_t channel = 0; channel < FTEPC485_CHANNEL_COUNT; channel++)
  {
    int32_t delta_pulse = data->pulse[channel] - handle->last_pulse[channel];

    data->speed_hundredths_mm_s[channel] =
        Ftepc485_DeltaToSpeedHundredthsMmS(channel, delta_pulse, delta_ms);
    handle->last_pulse[channel] = data->pulse[channel];
  }
  handle->last_tick_ms = now_ms;

  return FTEPC485_OK;
}
