/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "oled_ssd1306.h"
#include "pca9685.h"
#include "truck_control.h"
#include "truck_receiver.h"

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define USART2_DMA_RX_SIZE 256U
#define STATUS_TX_BUFFER_SIZE 192U
#define OLED_UPDATE_PERIOD_MS 500U
#define OLED_PAGE_PERIOD_MS 2000U
#define OLED_RETRY_PERIOD_MS 1000U
#define OLED_LINE_SIZE 22U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

static uint8_t usart2_dma_rx[USART2_DMA_RX_SIZE];
static TruckReceiver truck_receiver;
static TruckCommand latest_command;
static TruckOutputs truck_outputs;
static uint8_t pca9685_ready;
static uint8_t control_received;
static uint8_t control_failsafe_active;
static uint8_t system_ready;
static uint8_t usart2_rx_ready;
static uint8_t last_json_valid;
static uint8_t oled_ready;
static uint8_t oled_page;
static uint32_t last_valid_control_tick;
static uint32_t valid_frame_count;
static uint32_t invalid_frame_count;
static uint32_t last_oled_update_tick;
static uint32_t last_oled_page_tick;
static uint32_t last_oled_retry_tick;
static volatile uint8_t usart2_restart_requested;
static volatile uint8_t usart2_tx_busy;
static char status_tx_buffer[STATUS_TX_BUFFER_SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

static HAL_StatusTypeDef USART2_StartReceiveToIdle(void);
static HAL_StatusTypeDef ApplyServoTargets(
    const TruckOutputs *outputs);
static void SendControlStatus(const char *status,
                              HAL_StatusTypeDef i2c_status);
static void ProcessReceivedFrames(void);
static void ServiceControlFailsafe(void);
static void ServiceUsart2Restart(void);
static void ServiceOled(uint32_t now_ms);
static int32_t TruckAxisToMilli(float value);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static HAL_StatusTypeDef USART2_StartReceiveToIdle(void)
{
  HAL_StatusTypeDef status;

  status = HAL_UARTEx_ReceiveToIdle_DMA(&huart2, usart2_dma_rx,
                                       sizeof(usart2_dma_rx));
  if (status == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
    usart2_rx_ready = 1U;
  }
  else
  {
    usart2_rx_ready = 0U;
  }
  return status;
}

static HAL_StatusTypeDef ApplyServoTargets(
    const TruckOutputs *outputs)
{
  uint8_t channel;

  if (outputs == NULL)
  {
    return HAL_ERROR;
  }

  if (pca9685_ready == 0U)
  {
    if (Pca9685_Init(&hi2c1) != HAL_OK)
    {
      return HAL_ERROR;
    }
    pca9685_ready = 1U;
  }

  for (channel = 0U; channel < TRUCK_OUTPUT_CHANNELS; ++channel)
  {
    if (!TruckControl_IsActiveChannel(channel))
    {
      continue;
    }

    if (Pca9685_SetPwm(&hi2c1, channel, 0U,
                       outputs->pwm_count[channel]) != HAL_OK)
    {
      pca9685_ready = 0U;
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}

static void SendControlStatus(const char *status,
                              HAL_StatusTypeDef i2c_status)
{
  int length;

  if ((status == NULL) || (usart2_tx_busy != 0U))
  {
    return;
  }

  length = snprintf(
      status_tx_buffer, sizeof(status_tx_buffer),
      "{\"type\":\"truck\",\"status\":\"%s\",\"i2c\":%u,"
      "\"steering_deg\":%d,\"axis0_milli\":%ld,\"axis1_milli\":%ld,"
      "\"drive\":%d,\"lift\":%d,\"pwm\":[%u,%u,%u,%u]}\r\n",
      status,
      (i2c_status == HAL_OK) ? 1U : 0U,
      (int)truck_outputs.steering_deg,
      (long)TruckAxisToMilli(latest_command.drive_axis),
      (long)TruckAxisToMilli(latest_command.lift_axis),
      (int)truck_outputs.drive_percent,
      (int)truck_outputs.lift_percent,
      (unsigned int)truck_outputs.pwm_count[0],
      (unsigned int)truck_outputs.pwm_count[1],
      (unsigned int)truck_outputs.pwm_count[2],
      (unsigned int)truck_outputs.pwm_count[3]);
  if ((length <= 0) || ((size_t)length >= sizeof(status_tx_buffer)))
  {
    return;
  }

  usart2_tx_busy = 1U;
  if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)status_tx_buffer,
                           (uint16_t)length) != HAL_OK)
  {
    usart2_tx_busy = 0U;
  }
}

static void ProcessReceivedFrames(void)
{
  char frame[TRUCK_FRAME_MAX_LEN];

  while (TruckReceiver_Pop(&truck_receiver, frame, sizeof(frame)))
  {
    HAL_StatusTypeDef i2c_status;

    if (!TruckReceiver_ParseJson(frame, &latest_command))
    {
      invalid_frame_count++;
      last_json_valid = 0U;
      SendControlStatus("invalid_json", HAL_ERROR);
      continue;
    }

    valid_frame_count++;
    last_json_valid = 1U;
    TruckControl_MapRawCommand(&latest_command, &truck_outputs);
    i2c_status = ApplyServoTargets(&truck_outputs);
    last_valid_control_tick = HAL_GetTick();
    control_received = 1U;
    control_failsafe_active = 0U;
    SendControlStatus("applied", i2c_status);
  }
}

static void ServiceControlFailsafe(void)
{
  HAL_StatusTypeDef i2c_status;

  if ((control_received == 0U) || (control_failsafe_active != 0U) ||
      !TruckControl_IsTimedOut(HAL_GetTick(), last_valid_control_tick))
  {
    return;
  }

  TruckControl_SetNeutral(&truck_outputs);
  i2c_status = ApplyServoTargets(&truck_outputs);
  control_failsafe_active = 1U;
  SendControlStatus("failsafe", i2c_status);
}

static void ServiceUsart2Restart(void)
{
  if (usart2_restart_requested == 0U)
  {
    return;
  }

  usart2_restart_requested = 0U;
  (void)HAL_UART_AbortReceive(&huart2);
  if (USART2_StartReceiveToIdle() != HAL_OK)
  {
    usart2_restart_requested = 1U;
  }
}

static int32_t TruckAxisToMilli(float value)
{
  float scaled;

  if (value < -1.0f)
  {
    value = -1.0f;
  }
  else if (value > 1.0f)
  {
    value = 1.0f;
  }

  scaled = value * 1000.0f;
  return (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) :
                                          (scaled - 0.5f));
}

static void FormatAxis(char *buffer, size_t size, float value)
{
  const int32_t milli = TruckAxisToMilli(value);
  const uint32_t magnitude = (uint32_t)((milli < 0) ? -milli : milli);

  (void)snprintf(buffer, size, "%s%lu.%03lu",
                 (milli < 0) ? "-" : "",
                 (unsigned long)(magnitude / 1000U),
                 (unsigned long)(magnitude % 1000U));
}

static void OledDrawStatusPage(void)
{
  char line[OLED_LINE_SIZE];

  OledSsd1306_WriteText(0U, 0U, system_ready ? "BOOT:OK" : "BOOT:NO");
  OledSsd1306_WriteText(1U, 0U, usart2_rx_ready ? "UART:OK" : "UART:NO");
  OledSsd1306_WriteText(2U, 0U,
                        (control_received && !control_failsafe_active) ?
                        "RX:OK" : "RX:NO");
  OledSsd1306_WriteText(3U, 0U,
                        last_json_valid ? "PARSE:OK" : "PARSE:NO");
  OledSsd1306_WriteText(4U, 0U, pca9685_ready ? "PCA:OK" : "PCA:NO");
  OledSsd1306_WriteText(5U, 0U, "OLED:OK");

  (void)snprintf(line, sizeof(line), "GOOD:%lu",
                 (unsigned long)valid_frame_count);
  OledSsd1306_WriteText(6U, 0U, line);
  (void)snprintf(line, sizeof(line), "BAD:%lu DROP:%lu",
                 (unsigned long)invalid_frame_count,
                 (unsigned long)truck_receiver.dropped_frames);
  OledSsd1306_WriteText(7U, 0U, line);
}

static void OledDrawDataPage(void)
{
  char line[OLED_LINE_SIZE];
  char axis[10];

  FormatAxis(axis, sizeof(axis), latest_command.steering);
  (void)snprintf(line, sizeof(line), "STR:%s", axis);
  OledSsd1306_WriteText(0U, 0U, line);

  FormatAxis(axis, sizeof(axis), latest_command.drive_axis);
  (void)snprintf(line, sizeof(line), "AX0:%s", axis);
  OledSsd1306_WriteText(1U, 0U, line);

  FormatAxis(axis, sizeof(axis), latest_command.lift_axis);
  (void)snprintf(line, sizeof(line), "AX1:%s", axis);
  OledSsd1306_WriteText(2U, 0U, line);

  OledSsd1306_WriteText(3U, 0U, "TCA:OK");
  (void)snprintf(line, sizeof(line), "DEG:%d",
                 (int)truck_outputs.steering_deg);
  OledSsd1306_WriteText(4U, 0U, line);
  (void)snprintf(line, sizeof(line), "DRV:%d",
                 (int)truck_outputs.drive_percent);
  OledSsd1306_WriteText(5U, 0U, line);
  (void)snprintf(line, sizeof(line), "LFT:%d",
                 (int)truck_outputs.lift_percent);
  OledSsd1306_WriteText(6U, 0U, line);
  (void)snprintf(line, sizeof(line), "FS:%u I2C:%u",
                 control_failsafe_active, pca9685_ready);
  OledSsd1306_WriteText(7U, 0U, line);
}

static void ServiceOled(uint32_t now_ms)
{
  if (oled_ready == 0U)
  {
    if ((uint32_t)(now_ms - last_oled_retry_tick) < OLED_RETRY_PERIOD_MS)
    {
      return;
    }

    last_oled_retry_tick = now_ms;
    if (OledSsd1306_Init(&hi2c2) != HAL_OK)
    {
      return;
    }
    oled_ready = 1U;
    last_oled_update_tick = now_ms - OLED_UPDATE_PERIOD_MS;
    last_oled_page_tick = now_ms;
  }

  if ((uint32_t)(now_ms - last_oled_page_tick) >= OLED_PAGE_PERIOD_MS)
  {
    oled_page ^= 1U;
    last_oled_page_tick = now_ms;
  }

  if ((uint32_t)(now_ms - last_oled_update_tick) < OLED_UPDATE_PERIOD_MS)
  {
    return;
  }
  last_oled_update_tick = now_ms;

  OledSsd1306_Clear();
  if (oled_page == 0U)
  {
    OledDrawStatusPage();
  }
  else
  {
    OledDrawDataPage();
  }

  if (OledSsd1306_Update() != HAL_OK)
  {
    oled_ready = 0U;
    last_oled_retry_tick = now_ms;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  TruckReceiver_Init(&truck_receiver);
  TruckControl_SetNeutral(&truck_outputs);
  if (USART2_StartReceiveToIdle() != HAL_OK)
  {
    Error_Handler();
  }
  last_oled_retry_tick = HAL_GetTick();
  oled_ready = (OledSsd1306_Init(&hi2c2) == HAL_OK) ? 1U : 0U;
  last_oled_update_tick = HAL_GetTick() - OLED_UPDATE_PERIOD_MS;
  last_oled_page_tick = HAL_GetTick();
  system_ready = 1U;
  SendControlStatus("ready", ApplyServoTargets(&truck_outputs));

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ProcessReceivedFrames();
    ServiceControlFailsafe();
    ServiceUsart2Restart();
    ServiceOled(HAL_GetTick());

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    usart2_tx_busy = 0U;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART2)
  {
    TruckReceiver_FeedFromIsr(&truck_receiver, usart2_dma_rx, Size);
    if (USART2_StartReceiveToIdle() != HAL_OK)
    {
      usart2_restart_requested = 1U;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    usart2_rx_ready = 0U;
    usart2_restart_requested = 1U;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
