/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <can.h>
#include"apps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// boiade
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac_ch1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
const uint16_t DAC_30 = 865; // 1439 - OLD;
const uint16_t DAC_0 = 500; // 1111 - OLD;
uint32_t ADC_value[2];
uint32_t dacValue = DAC_0;
int gear = 0;
int nMot = 0;

int isLaunchControlSetCnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

typedef enum ClutchState {
  START,
  PULL,
  PULLED,
  GEAR_NOT_SET,
  GEAR_CHANGING,
  GEAR_CHANGED,
  GEAR_NOT_CHANGED,
  LAUNCH_CONTROL_SET,
  APPS_30,
  RELEASE,
  RELEASED,
  DONE,
} ClutchState;

typedef enum GearN {
  GEAR_N_OFF = GPIO_PIN_RESET,
  GEAR_N_ON = GPIO_PIN_SET,
} GearN;

static ClutchState state = START;
static HalStatus status = HAL_OK;


static bool changeGearStop = false;
static bool sendLaunch = false;
static bool firstrun=0;


static uint8_t changeGear(bool reset) {
  static int start = 0;
  if (reset)
    start = uwTick;

  if (changeGearStop || gear) {
    HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, GEAR_N_OFF);
    return true;
  }

  bool isWithin50ms = uwTick - start < 350;
  uint8_t out = isWithin50ms
    ? GEAR_N_ON
    : GEAR_N_OFF;

  HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, out);
  return out == GEAR_N_ON;
}



static bool waitMs(bool reset, int ms) {
  static int start = 0;
  if (reset) {
    start = uwTick;
  }

  return uwTick - start >= ms;
}

static uint16_t readGear(CanRxBuffer buffer) {
  return buffer[5] << 8 | buffer[4];
}

static uint16_t readRPM(CanRxBuffer buffer) {
  return buffer[1] << 8 | buffer[0];
}

static uint8_t readLaunch(CanRxBuffer buffer) {
  int bytes7_8 = (buffer[7] << 8 | buffer[6]);
  int isLaunchControlBitHigh = (bytes7_8 & 0x8) == 0x8;

  return isLaunchControlBitHigh ? +1 : -1;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



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
  MX_CAN_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, ADC_value, sizeof(ADC_value) / sizeof(*ADC_value));
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, &dacValue, 1, DAC_ALIGN_12B_R);
  HAL_TIM_Base_Start(&htim2);
  if (MMR_CAN_BasicSetupAndStart(&hcan) != HAL_OK)
    Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  MmrCanRxBuffer buffer = {};
  MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_CS_CLUTCH_PULL);
  MmrCanMessage msg = MMR_CAN_OutMessage(header);


  MmrCanPacket clutchPull = { .header.messageId = MMR_CAN_MESSAGE_ID_CS_CLUTCH_PULL };
  MmrCanPacket clutchRelease = { .header.messageId = MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE };
  MmrCanPacket clutchSetManual = { .header.messageId = MMR_CAN_MESSAGE_ID_CS_CLUTCH_SET_MANUAL };

  uint8_t _launchControlData[8] = {0x8};
  MmrCanPacket launchControl = {
      .data = _launchControlData,
      .length = 8,
      .noExtId = true,
      .header = MMR_CAN_HeaderFromBits(0x628),
  };

  int clutchMsgStart = uwTick;
  int clutchMsgCnt = 0;

  int launchControlStart = 0;
  int launchControlStop = 0;
  int launchControlCnt = 0;


  while (1)
  {
    int pendingMsgs = HAL_CAN_GetRxFifoFillLevel(&hcan, MMR_CAN_RX_FIFO);
    if (pendingMsgs > 0) {
      MMR_CAN_Receive(&hcan, &msg);

      switch (msg.header.messageId) {
      case MMR_CAN_MESSAGE_ID_ECU_ENGINE_FN1:
        gear = readGear(buffer);
        nMot = readRPM(buffer);
        break;

      case MMR_CAN_MESSAGE_ID_ECU_ENGINE_FN2:
        isLaunchControlSetCnt += readLaunch(buffer);
        break;
      }
    }

    if (nMot > 1000 && sendLaunch) {
      if (isLaunchControlSetCnt >= 2 || isLaunchControlSetCnt <= -2) {
        sendLaunch = false;
      }

      status = MMR_CAN_SendNoTamper(&hcan, launchControl);
    }

    switch (state) {
    case START:
    case PULL:
    	if (nMot > 1000) {
        if (uwTick - clutchMsgStart > 1) {
          status = MMR_CAN_Send(&hcan, clutchPull);
          clutchMsgStart = uwTick;
        }

        if (msg.header.messageId == MMR_CAN_MESSAGE_ID_CS_CLUTCH_PULL_OK) {
          sendLaunch = true;
          launchControlStart = uwTick;
          state = PULLED;
        }
    	}
      break;

    case PULLED:
      if (msg.header.messageId == MMR_CAN_MESSAGE_ID_ECU_ENGINE_FN2) {
        if (buffer[6]) {
          clutchMsgStart = uwTick;
          clutchMsgCnt = 0;
          state = GEAR_NOT_SET;
        }
      }
      break;

    case GEAR_NOT_SET:
      if (!gear)
        changeGear(true);

      state = GEAR_CHANGING;
      break;

    case GEAR_CHANGING:
      if (gear) {
        changeGearStop = true;
        changeGear(true);
        HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, GEAR_N_OFF);
        state = GEAR_CHANGED;
        break;
      }

      if (!gear || !changeGear(false)) {
        waitMs(true, 2000);
        state = GEAR_NOT_CHANGED;
        break;
      }

      break;

    case GEAR_NOT_CHANGED:
      if (waitMs(false, 2000)) {
        HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, GEAR_N_OFF);
        state = GEAR_NOT_SET;
      }

      break;

    case GEAR_CHANGED:
      HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, GEAR_N_OFF);
      waitMs(true, 1000);
      state = LAUNCH_CONTROL_SET;
      break;

    case LAUNCH_CONTROL_SET:
      dacValue = DAC_30;
      if (waitMs(false, 1000)) {
        state = APPS_30;
      }
      break;

    case APPS_30:
      if (uwTick - clutchMsgStart > 1) {
        status = MMR_CAN_Send(&hcan, clutchRelease);
        clutchMsgStart = uwTick;

        if (clutchMsgCnt++ >= 5 && nMot >= 6000) {
          state = RELEASE;
        }
      }

      break;

    case RELEASE:
      if (msg.header.messageId == MMR_CAN_MESSAGE_ID_CS_CLUTCH_RELEASE_OK) {
    	  firstrun=1;
        clutchMsgStart = uwTick;
        launchControlStop = uwTick;
        clutchMsgCnt = 0;
        launchControlCnt = 0;
        isLaunchControlSetCnt = 0;
        state = RELEASED;
      }
      break;

    case RELEASED:
      if (clutchMsgCnt < 5 && uwTick - clutchMsgStart > 5) {
        status = MMR_CAN_Send(&hcan, clutchSetManual);
        clutchMsgStart = uwTick;
        clutchMsgCnt++;
      }

      if (clutchMsgCnt >= 5) {
        state = DONE;
      }
      break;

    case DONE:
      break;
    }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC1;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV64;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 8;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GEAR_CHANGE_GPIO_Port, GEAR_CHANGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : GEAR_CHANGE_Pin */
  GPIO_InitStruct.Pin = GEAR_CHANGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GEAR_CHANGE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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

#ifdef  USE_FULL_ASSERT
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
