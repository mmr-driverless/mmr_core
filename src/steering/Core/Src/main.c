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
  * Pinout:
  * D2	CANTX	PA_12
  * D10	CANRX	PA_11
  * VIN	VIN		VIN
  * GND	GND		GND
  * A0	PULSE 	PA_0
  * A1	DIR 	PA_1
  * A2	ENB 	PA_3
  * A4	SPOT1 	PA_5
  * A5	SPOT2 	PA_6
  *
  * TIM2 generates PWM, TIM3 controls the frequency of the controller.
  * Callbacks should only be performed based on TIM3 ITs, while TIM2 parameters
  * are meant to be modified by the callbacks in order to control the motor.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <timing.h>
#include <can0.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct proportional_data{
	uint32_t left_y, right_y;
	float left_x, right_x;
};

struct lowpass_data{
	uint16_t  input, output;
	float cutoff_frequency, dt;
};

struct lowpass32_data{
	uint32_t  input, output;
	float cutoff_frequency, dt;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_SIZE 50
#define POSITION_SAMPLES_NUMBER 5

const float MAX_STEERING_ANGLE = 21.5f*6.5625f; // [deg]
const float ADC_MAX_VALUE = 1024.0f;
const float ADC_MAX_VOLTAGE = 3.6f; // [V]
const float DEGREES_PER_VOLT = 20.0f; // [deg/V]
const float VOLTAGE_RATIO = 3.3f/5.0f;
const float TOLERANCE = 5.0f; // [deg]
const float STEERING_RATIO = 6.5625f;
const float STD_DEVIATION_LIMIT = 2.5f;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
//volatile float degrees;
struct proportional_data p_data_angular_error, p_data_odometry_speed_1, p_data_odometry_speed_2;
struct lowpass_data lowpass_data;
struct lowpass32_data lowpass32_data;

float target_angle=0; // [deg]
float speed=10; // [m/s]

float current_angle=0; // [deg]
float tension=0; // [V]
float angular_error=0; // [deg]
uint16_t prescaler=100-1;
uint16_t ADC2_Value[ADC_SIZE];
uint16_t filtered_ADC = 0;
uint32_t target_psc = 500;
uint16_t position_samples[POSITION_SAMPLES_NUMBER];
uint16_t k=0;
uint16_t cansendflag=0;

float flag=0;
float sigma=0;
float centerOffset=2.62f-0.505f+0.4975f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void Autoset(){
	centerOffset=((filtered_ADC / ADC_MAX_VALUE) * ADC_MAX_VOLTAGE)/(VOLTAGE_RATIO);

	MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_ST_STEERING_CENTER_AUTOSET_OK);
  MmrCanMessage autosetOkMsg = MMR_CAN_OutMessage(header);

	MMR_CAN_Send(&can0, &autosetOkMsg);
}

float Average(uint16_t p[], int length){
    float avg=0;
    for (int i =0; i<length; i++){
        avg+=p[i];
    }
    return avg/(length);
}

float StandardDeviation(uint16_t p[], int length){
    float avg=Average(p, length);
    float sigma=0;
    for (int i =0; i<length; i++){
        sigma+=pow((float)(p[i]-avg), 2);
    }
    return sqrt(sigma/length);

}

uint32_t Proportional(float error, struct proportional_data p_data){
	float absolute_error=fabsf(error);
	if(absolute_error<p_data.left_x){
		return p_data.left_y;
	} else if(absolute_error>p_data.right_x){
		return p_data.right_y;
	} else{
		float proportional_slope=(p_data.left_y-p_data.right_y)/(p_data.left_x-p_data.right_x);
		return (proportional_slope*absolute_error)+p_data.left_y-(proportional_slope*p_data.left_x);
	}
}

static void steer(
  GPIO_PinState dirPinExpected,
  GPIO_PinState dirPinWrite
) {
  if (HAL_GPIO_ReadPin(DIR_GPIO_Port, DIR_Pin) == dirPinExpected){
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, dirPinWrite);
    target_psc=Proportional(speed, p_data_odometry_speed_1);
    lowpass32_data.output=target_psc;
  }

  TIM2->CCR1 = 50;
}

static void steerRight() {
  steer(GPIO_PIN_SET, GPIO_PIN_RESET);
}

static void steerLeft() {
  steer(GPIO_PIN_RESET, GPIO_PIN_SET);
}

uint16_t Lowpass(uint16_t input, struct lowpass_data *lp_data){
	lp_data->input=input;
	return lp_data->output += (lp_data->input - lp_data->output) * (1-exp(-lp_data->dt * lp_data->cutoff_frequency));
}

uint32_t Lowpass32(uint32_t input, struct lowpass32_data *lp32_data){
	lp32_data->input=input;
	float exp_portion=1.f-exp(-lp32_data->dt * lp32_data->cutoff_frequency);
	float input_minus_output=(float)lp32_data->input - (float)lp32_data->output;
	return lp32_data->output = (uint32_t)((float)lp32_data->output + (float)(input_minus_output * exp_portion));
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback
  if (htim == &htim3)
  {
	filtered_ADC =Lowpass(ADC2_Value[0], &lowpass_data);

    tension = ((filtered_ADC / ADC_MAX_VALUE) * ADC_MAX_VOLTAGE)/(VOLTAGE_RATIO) - centerOffset;

    current_angle = tension * DEGREES_PER_VOLT * 4.0f;

	//current_angle = 0.72f*filtered_ADC-284.4f;
    angular_error = -(target_angle - current_angle); // We calculate the error compared to the target

    if(k>=POSITION_SAMPLES_NUMBER-1){
    	k=0;
    } else k++;
    position_samples[k]=filtered_ADC;
    sigma = StandardDeviation(position_samples, POSITION_SAMPLES_NUMBER);
    if(sigma<STD_DEVIATION_LIMIT) {
    	target_psc=Proportional(speed, p_data_odometry_speed_1);
    	lowpass32_data.output=target_psc;
    }


    if (angular_error > TOLERANCE) {
      steerRight();
      p_data_angular_error.left_y=Proportional(speed, p_data_odometry_speed_1);
      p_data_angular_error.right_y=Proportional(speed, p_data_odometry_speed_2);
      target_psc=Proportional(angular_error, p_data_angular_error);
      TIM2->PSC = Lowpass32(Proportional(angular_error, p_data_angular_error), &lowpass32_data);
    }
    else if (angular_error < -TOLERANCE) {
      steerLeft();
      p_data_angular_error.left_y=Proportional(speed, p_data_odometry_speed_1);
      p_data_angular_error.right_y=Proportional(speed, p_data_odometry_speed_2);
      target_psc=Proportional(angular_error, p_data_angular_error);
      TIM2->PSC = Lowpass32(Proportional(angular_error, p_data_angular_error), &lowpass32_data);
    }
    else {
      target_psc=Proportional(speed, p_data_odometry_speed_1);
      lowpass32_data.output=target_psc;
      TIM2->PSC=target_psc;
      TIM2->CCR1 = 0;
    }

    // CAN LOGGING
    if (cansendflag==5){
    	float current_steering_angle=current_angle/STEERING_RATIO;
      //uint8_t current_steering_angle = 8;
    	uint8_t *data = (uint8_t*)(&current_steering_angle);
    	MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_ST_CURRENT_ANGLE);
      MmrCanMessage currentAngleMsg = MMR_CAN_OutMessage(header);
      MMR_CAN_MESSAGE_SetPayload(&currentAngleMsg, data, sizeof(float));

      MMR_CAN_Send(&can0, &currentAngleMsg);
      cansendflag=0;
    } else {
    	cansendflag++;
    }


  }
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
  MX_ADC2_Init();

  MX_CAN_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  if (!MMR_CAN0_Start(&hcan)) {
    Error_Handler();
  }

  MMR_SetTickProvider(HAL_GetTick);

  HAL_ADC_Start_DMA(&hadc2, (uint16_t*)ADC2_Value, ADC_SIZE);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  MmrCanBuffer buffer = {};
  MmrCanMessage message = {
    .payload = buffer,
  };

  // Definition of proportional parameters
  p_data_angular_error.left_x=0.0f;
  p_data_angular_error.left_y=500;
  p_data_angular_error.right_x=10.0f;
  p_data_angular_error.right_y=200;

  p_data_odometry_speed_1.left_x=0.5f;
  p_data_odometry_speed_1.left_y=4000;
  p_data_odometry_speed_1.right_x=1.0f;
  p_data_odometry_speed_1.right_y=500;

  p_data_odometry_speed_2.left_x=0.5f;
  p_data_odometry_speed_2.left_y=3000;
  p_data_odometry_speed_2.right_x=1.0f;
  p_data_odometry_speed_2.right_y=200;

  // Definition of lowpass parameters
  lowpass_data.input=ADC2_Value[0];
  lowpass_data.output=ADC2_Value[0];
  lowpass_data.cutoff_frequency=20.0f; // [rad/s]
  lowpass_data.dt=0.0125f; // TIM3 time period

  lowpass32_data.input=500;
  lowpass32_data.output=500;
  lowpass32_data.cutoff_frequency=15.0f; // [rad/s]
  lowpass32_data.dt=0.0125f; // TIM3 time period

  for (int i=0; i<POSITION_SAMPLES_NUMBER; i++){
	  position_samples[i]=0;
  }

  HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_SET);
  while (1)
  {
	    if (MMR_CAN_ReceiveAsync(&can0, &message) == MMR_TASK_COMPLETED) {
	      MmrCanHeader header = MMR_CAN_MESSAGE_GetHeader(&message);
	      switch (header.messageId) {
	      case MMR_CAN_MESSAGE_ID_D_STEERING_ANGLE:
	        target_angle = (*(float*)buffer) * STEERING_RATIO;
	        break;

	      case MMR_CAN_MESSAGE_ID_D_SPEED_ODOMETRY:
	    	speed = (*(float*)buffer);
	    	break;

	      case MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ERROR_LEFT_X:
	    	  p_data_angular_error.left_x=(*(float*)buffer);
	    	  break;

	      case MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ERROR_RIGHT_X:
	    	  p_data_angular_error.right_x=(*(float*)buffer);
	    	  break;

	      case MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MIN_SPEED_LEFT_X:
	    	  p_data_odometry_speed_1.left_x=(*(float*)buffer);
	    	  p_data_odometry_speed_2.left_x=(*(float*)buffer);
	    	  break;

	      case MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MIN_SPEED_LEFT_Y:
	    	  p_data_odometry_speed_1.left_y=(*(uint32_t*)buffer);
	    	  break;

	      case MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MIN_SPEED_RIGHT_X:
	    	  p_data_odometry_speed_1.right_x=(*(float*)buffer);
	    	  p_data_odometry_speed_2.right_x=(*(float*)buffer);
	    	  break;

	      case MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MIN_SPEED_RIGHT_Y:
	    	  p_data_odometry_speed_1.right_y=(*(uint32_t*)buffer);
	    	  break;

	      case MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MAX_SPEED_LEFT_Y:
	    	  p_data_odometry_speed_2.left_y=(*(uint32_t*)buffer);
	    	  break;

	      case MMR_CAN_MESSAGE_ID_ST_PROPORTIONAL_ODOMETRY_MAX_SPEED_RIGHT_Y:
	    	  p_data_odometry_speed_2.right_y=(*(uint32_t*)buffer);
	    	  break;

	      case MMR_CAN_MESSAGE_ID_ST_RISE_CUTOFF_FREQUENCY:
	    	  lowpass32_data.cutoff_frequency=(*(float*)buffer);
	    	  break;

	      case MMR_CAN_MESSAGE_ID_ST_STEERING_CENTER_AUTOSET:
	      	  Autoset();
	      	  break;
	      }

	    }
	    if(target_angle>MAX_STEERING_ANGLE){
	    	target_angle=MAX_STEERING_ANGLE;
	    } else if(target_angle < -MAX_STEERING_ANGLE){
	    	target_angle = -MAX_STEERING_ANGLE;
	    }

	    /*
	    if(current_angle>120){
	    	target_angle=-140;
	    } else if (current_angle<-120){
	    	target_angle=140;
	    }
	    */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    flag++;
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_10B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hcan.Init.Prescaler = 4;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 2000-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 2000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : DIR_Pin ENB_Pin */
  GPIO_InitStruct.Pin = DIR_Pin|ENB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
