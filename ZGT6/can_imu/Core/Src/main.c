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
#include "imu_parser.h"
#include "imu_oled_format.h"
#include "oled_ssd1306.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define IMU_CAN_ID 0x121U
#define CAN_RX_QUEUE_LEN 128U
#define IMU_FRAME_GAP_MS 2000U
#define IMU_STALE_MS 500U
#define OLED_REFRESH_MS 200U
#define OLED_ROW_INTERVAL_MS 25U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
typedef struct
{
  uint8_t length;
  uint8_t data[8];
  uint32_t tick_ms;
} CanRxChunk;

static CanRxChunk can_rx_queue[CAN_RX_QUEUE_LEN];
static volatile uint16_t can_rx_head;
static volatile uint16_t can_rx_tail;
static volatile uint8_t can_rx_overflow;
static ImuParser imu_parser;
static ImuSample imu_latest;
static uint8_t imu_valid;
static uint8_t oled_ready;
static uint32_t imu_last_valid_ms;
static uint32_t last_can_frame_ms;
static uint32_t last_oled_refresh_ms;
static uint32_t last_oled_row_ms;
static uint32_t last_oled_retry_ms;
static uint8_t next_oled_row;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_IMU_Start(void);
static void CAN_IMU_Process(void);
static void OLED_IMU_Draw(uint32_t now_ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void CAN_IMU_Start(void)
{
  CAN_FilterTypeDef filter = {0};
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (uint16_t)(IMU_CAN_ID << 5);
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5);
  filter.FilterMaskIdLow = 0x0006U; /* Standard ID and data frame. */
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK ||
      HAL_CAN_Start(&hcan1) != HAL_OK ||
      HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];

  if (hcan->Instance != CAN1)
  {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    uint16_t next;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
      can_rx_overflow = 1U;
      break;
    }
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
        header.StdId != IMU_CAN_ID || header.DLC == 0U || header.DLC > 8U)
    {
      continue;
    }

    next = (uint16_t)((can_rx_head + 1U) % CAN_RX_QUEUE_LEN);
    if (next == can_rx_tail)
    {
      can_rx_overflow = 1U;
      continue;
    }

    can_rx_queue[can_rx_head].length = (uint8_t)header.DLC;
    memcpy(can_rx_queue[can_rx_head].data, data, header.DLC);
    can_rx_queue[can_rx_head].tick_ms = HAL_GetTick();
    __DMB();
    can_rx_head = next;
  }
}

static void CAN_IMU_Process(void)
{
  if (can_rx_overflow != 0U)
  {
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    can_rx_head = 0U;
    can_rx_tail = 0U;
    can_rx_overflow = 0U;
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    imu_parser_init(&imu_parser);
    imu_valid = 0U;
    last_can_frame_ms = 0U;
  }

  while (can_rx_tail != can_rx_head)
  {
    const CanRxChunk *chunk = &can_rx_queue[can_rx_tail];
    if (last_can_frame_ms != 0U &&
        (uint32_t)(chunk->tick_ms - last_can_frame_ms) > IMU_FRAME_GAP_MS)
    {
      imu_parser_init(&imu_parser);
    }
    last_can_frame_ms = chunk->tick_ms;

    /* Assumes ID 0x121 DATA is an ordered, unmodified IMU byte stream. */
    if (imu_parser_feed(&imu_parser, chunk->data, chunk->length, &imu_latest))
    {
      imu_valid = 1U;
      imu_last_valid_ms = chunk->tick_ms;
    }
    __DMB();
    can_rx_tail = (uint16_t)((can_rx_tail + 1U) % CAN_RX_QUEUE_LEN);
  }
}

static void OLED_IMU_Row(uint8_t row, const char *label, float value)
{
  char line[22];
  imu_oled_format_row(line, sizeof(line), label, value);
  OledSsd1306_WriteText(row, 0U, line);
}

static void OLED_IMU_Draw(uint32_t now_ms)
{
  OledSsd1306_Clear();
  if (imu_valid == 0U ||
      (uint32_t)(now_ms - imu_last_valid_ms) > IMU_STALE_MS)
  {
    OledSsd1306_WriteText(0U, 0U, "CAN IMU 0x121");
    OledSsd1306_WriteText(2U, 0U, "WAIT IMU DATA");
  }
  else
  {
    OledSsd1306_WriteText(0U, 0U, "ANGLE deg");
    OLED_IMU_Row(1U, "Roll", imu_latest.roll_deg);
    OLED_IMU_Row(2U, "Pitch", imu_latest.pitch_deg);
    OLED_IMU_Row(3U, "Yaw", imu_latest.yaw_deg);
    OledSsd1306_WriteText(4U, 0U, "RATE deg/s");
    OLED_IMU_Row(5U, "Gx", imu_latest.gx_deg_s);
    OLED_IMU_Row(6U, "Gy", imu_latest.gy_deg_s);
    OLED_IMU_Row(7U, "Gz", imu_latest.gz_deg_s);
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
  MX_CAN1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  imu_parser_init(&imu_parser);
  oled_ready = (OledSsd1306_Init(&hi2c2) == HAL_OK) ? 1U : 0U;
  last_oled_retry_ms = HAL_GetTick();
  CAN_IMU_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now_ms;
    CAN_IMU_Process();
    now_ms = HAL_GetTick();
    if (oled_ready == 0U &&
        (uint32_t)(now_ms - last_oled_retry_ms) >= 1000U)
    {
      oled_ready = (OledSsd1306_Init(&hi2c2) == HAL_OK) ? 1U : 0U;
      last_oled_retry_ms = now_ms;
    }
    if (oled_ready != 0U &&
        (uint32_t)(now_ms - last_oled_refresh_ms) >= OLED_REFRESH_MS)
    {
      OLED_IMU_Draw(now_ms);
      last_oled_refresh_ms = now_ms;
    }
    if (oled_ready != 0U &&
        (uint32_t)(now_ms - last_oled_row_ms) >= OLED_ROW_INTERVAL_MS)
    {
      if (OledSsd1306_UpdateRow(next_oled_row) != HAL_OK)
      {
        oled_ready = 0U;
        last_oled_retry_ms = now_ms;
      }
      next_oled_row = (uint8_t)((next_oled_row + 1U) % OLED_SSD1306_ROWS);
      last_oled_row_ms = now_ms;
    }
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
