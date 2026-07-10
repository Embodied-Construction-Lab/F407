#ifndef FTEPC_RS485_H
#define FTEPC_RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include <stdint.h>

#define FTEPC485_CHANNEL_COUNT 3U
#define FTEPC485_HUNDREDTHS_MM_PER_PULSE_NUM 25
#define FTEPC485_HUNDREDTHS_MM_PER_PULSE_DEN 2

typedef enum
{
  FTEPC485_OK = 0,
  FTEPC485_ERROR_UART,
  FTEPC485_ERROR_CRC,
  FTEPC485_ERROR_FRAME,
  FTEPC485_ERROR_ARGUMENT
} Ftepc485_Status;

typedef struct
{
  int32_t pulse[FTEPC485_CHANNEL_COUNT];
  int32_t length_hundredths_mm[FTEPC485_CHANNEL_COUNT];
  int32_t speed_hundredths_mm_s[FTEPC485_CHANNEL_COUNT];
} Ftepc485_Data;

typedef struct
{
  UART_HandleTypeDef *uart;
  GPIO_TypeDef *de_port;
  uint16_t de_pin;
  uint8_t slave_address;
  uint32_t timeout_ms;
  int32_t last_pulse[FTEPC485_CHANNEL_COUNT];
  uint32_t last_tick_ms;
  uint8_t has_last_sample;
} Ftepc485_Handle;

void Ftepc485_Init(Ftepc485_Handle *handle,
                   UART_HandleTypeDef *uart,
                   GPIO_TypeDef *de_port,
                   uint16_t de_pin,
                   uint8_t slave_address);

Ftepc485_Status Ftepc485_ReadThreePulses(Ftepc485_Handle *handle,
                                         Ftepc485_Data *data);

Ftepc485_Status Ftepc485_Poll(Ftepc485_Handle *handle,
                              Ftepc485_Data *data,
                              uint32_t now_ms);

void Ftepc485_ConvertPulses(const int32_t pulses[3], Ftepc485_Data *data);
int32_t Ftepc485_DeltaToSpeedHundredthsMmS(uint8_t channel,
                                           int32_t delta_pulse,
                                           uint32_t delta_ms);
uint16_t Ftepc485_Crc16(const uint8_t *data, uint16_t length);
Ftepc485_Status Ftepc485_ParseThreePulseResponse(const uint8_t *response,
                                                 uint16_t length,
                                                 uint8_t slave_address,
                                                 int32_t pulses[3]);

#ifdef __cplusplus
}
#endif

#endif
