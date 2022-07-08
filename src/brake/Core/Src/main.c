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

enum Mode {
	RUN,
	CHECK_ASB,
};

typedef enum Mode Mode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_SIZE 50
#define PRESSURE_SAMPLES_NUMBER 5

const float ADC_MAX_VALUE = 1024.0f;
const float ADC_MAX_VOLTAGE = 3.6f; // [V]
const float VOLTAGE_RATIO = 3.3f/5.0f;
const float TOLERANCE = 0.5f; // [bar]
const float STD_DEVIATION_LIMIT = 2.5f;
const float AMBIENT_PRESSURE_VOLTAGE = 0.5f; // [V]
const float BARS_PER_VOLT = 40.0f; // [bar/V]
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
//volatile float degrees;
struct proportional_data p_data_pressure_error;
struct lowpass_data lowpass_data_pressure, lowpass_data_angle;
struct lowpass32_data lowpass32_data;

float target_pressure=0; // [bar]
float current_pressure=0; // [bar]
float tension=0; // [V]
float pressure_error=0; // [bar]
uint16_t prescaler=100-1;
uint16_t ADC2_Value[ADC_SIZE];
uint16_t filtered_ADC_pressure = 0;
uint16_t filtered_ADC_angle = 0;
uint32_t target_psc = 400;
uint16_t pressure_samples[PRESSURE_SAMPLES_NUMBER];
uint16_t k=0;
uint16_t cansendflag=0;

float flag=0;
float sigma=0;
uint16_t released_pedal_ADC=0; // must be set based on filtered_ADC_angle evaluated in fully released conditions
uint16_t released_pedal_ADC_tolerance=20;
float max_pressure = 6.0f; // [bar]

Mode mode = RUN;
uint16_t asbCheckCycles = 0;

bool isActive = true;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM7_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
void ASB_Check(){
	if(current_pressure >= target_pressure - TOLERANCE){
	    		uint8_t *data = (uint8_t*)(1);

	    		MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_AS_ASB_CHECK);
	    		MmrCanMessage asbCheckMessage = MMR_CAN_OutMessage(header);
	    		MMR_CAN_MESSAGE_SetPayload(&asbCheckMessage, data, sizeof(uint8_t));

	    		MMR_CAN_Send(&can0, &asbCheckMessage);

	    		mode = RUN;
	    		target_pressure=0.0f;
	    		asbCheckCycles=0;
	    	} else {
	    		asbCheckCycles++;
	    	}

	    	if(asbCheckCycles>=400){
	    		uint8_t *data = (uint8_t*)(0);

	    		MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_AS_ASB_CHECK);
	    		MmrCanMessage asbCheckMessage = MMR_CAN_OutMessage(header);
	    		MMR_CAN_MESSAGE_SetPayload(&asbCheckMessage, data, sizeof(uint8_t));

	    		MMR_CAN_Send(&can0, &asbCheckMessage);

	    		mode = RUN;
	    		target_pressure=0.0f;
	    		asbCheckCycles=0;
	    	}

}

void Autoset(){
	released_pedal_ADC=filtered_ADC_angle;

	MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_BRK_ZERO_PRESSURE_AUTOSET_OK);
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

static void brake(
  GPIO_PinState dirPinExpected,
  GPIO_PinState dirPinWrite
) {
  if (HAL_GPIO_ReadPin(DIR_GPIO_Port, DIR_Pin) == dirPinExpected){
    HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, dirPinWrite);
    target_psc=p_data_pressure_error.left_y;
    lowpass32_data.output=target_psc;
  }

  TIM2->CCR1 = 50;
}

static void press() {
	brake(GPIO_PIN_SET, GPIO_PIN_RESET);
}

static void release() {

	/*
	 *  // SAFETY CHECK WITH POTENTIOMETER
	 *
	if(fabsf((float)filtered_ADC_angle -(float)released_pedal_ADC) < (float)released_pedal_ADC_tolerance){
		TIM2-CCR1=0;
	} else {
		brake(GPIO_PIN_RESET, GPIO_PIN_SET);
	}
	*/

	// NO SAFETY CHECK - NEEDS REPLACEMENT
	brake(GPIO_PIN_RESET, GPIO_PIN_SET);
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
  if (htim == &htim7)
  {
	filtered_ADC_pressure=Lowpass(ADC2_Value[0], &lowpass_data_pressure); // A4 PA5 SPOT1
	filtered_ADC_angle=Lowpass(ADC2_Value[1], &lowpass_data_angle); //A5 PA6 SPOT2

    tension = ((filtered_ADC_pressure / ADC_MAX_VALUE) * ADC_MAX_VOLTAGE)/(VOLTAGE_RATIO);

    //current_pressure = (tension - AMBIENT_PRESSURE_VOLTAGE) * BARS_PER_VOLT;

    pressure_error = (target_pressure - current_pressure); // We calculate the error compared to the target

    if(k>=PRESSURE_SAMPLES_NUMBER-1){
    	k=0;
    } else k++;
    pressure_samples[k]=current_pressure;
    sigma = StandardDeviation(pressure_samples, PRESSURE_SAMPLES_NUMBER);
    if(sigma<STD_DEVIATION_LIMIT) {
    	target_psc=p_data_pressure_error.left_y;
    	lowpass32_data.output=target_psc;
    }

    if (pressure_error > TOLERANCE) {
      press();
      target_psc=Proportional(pressure_error, p_data_pressure_error);
      TIM2->PSC = Lowpass32(target_psc, &lowpass32_data);
    }
    else if (pressure_error < -TOLERANCE) {
      release();
      target_psc=Proportional(pressure_error, p_data_pressure_error);
      TIM2->PSC = Lowpass32(target_psc, &lowpass32_data);
    }
    else {
      target_psc=p_data_pressure_error.left_y;
      lowpass32_data.output=target_psc;
      TIM2->PSC=target_psc;
      TIM2->CCR1 = 0;
    }

    //if (filtered_ADC_angle) set the duty cycle to zero if going too far back
    // CAN LOGGING - SKIPPED AS CURRENTLY BRAKE PRESSURE IS PUBLISHED BY ECU @ID199
    /*
    if (cansendflag==5){
    	    uint8_t *data = (uint8_t*)(&current_pressure);

    	    MmrCanHeader header = MMR_CAN_NormalHeader(MMR_CAN_MESSAGE_ID_BRK_CURRENT_PRESSURE);
    	    MmrCanMessage currentPressureMsg = MMR_CAN_OutMessage(header);
    	    MMR_CAN_MESSAGE_SetPayload(&currentPressureMsg, data, sizeof(float));

    	    MMR_CAN_Send(&can0, &currentPressureMsg);

    	    cansendflag=0;
    } else {
    	cansendflag++;
    }
	*/
    if (mode == CHECK_ASB){
    	ASB_Check();
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
    if (!MMR_CAN0_Start(&hcan1)) {
      Error_Handler();
    }

    MMR_SetTickProvider(HAL_GetTick);

    HAL_ADC_Start_DMA(&hadc1, (uint16_t*)ADC2_Value, ADC_SIZE);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    MmrCanBuffer buffer = {};
    MmrCanMessage message = {
      .payload = buffer,
    };

    // Definition of proportional parameters
    p_data_pressure_error.left_x=0.0f;
    p_data_pressure_error.left_y=400;
    p_data_pressure_error.right_x=2.0f;
    p_data_pressure_error.right_y=100;

    // Definition of lowpass parameters
    lowpass_data_pressure.input=ADC2_Value[0];
    lowpass_data_pressure.output=ADC2_Value[0];
    lowpass_data_pressure.cutoff_frequency=20.0f; // [rad/s]
    lowpass_data_pressure.dt=0.0125f; // TIM3 time period

    lowpass_data_angle.input=ADC2_Value[1];
    lowpass_data_angle.output=ADC2_Value[1];
    lowpass_data_angle.cutoff_frequency=20.0f; // [rad/s]
    lowpass_data_angle.dt=0.0125f; // TIM3 time period

    lowpass32_data.input=400;
    lowpass32_data.output=400;
    lowpass32_data.cutoff_frequency=15.0f; // [rad/s]
    lowpass32_data.dt=0.0125f; // TIM3 time period

    for (int i=0; i<PRESSURE_SAMPLES_NUMBER; i++){
    	pressure_samples[i]=0;
    }

    if(isActive) {
    	HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_SET);
    } else {
    	HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_RESET);
    }
  while (1)
  {

	  if (MMR_CAN_ReceiveAsync(&can0, &message) == MMR_TASK_COMPLETED) {
	  	      MmrCanHeader header = MMR_CAN_MESSAGE_GetHeader(&message);

	  	      switch (header.messageId) {
	  	      case MMR_CAN_MESSAGE_ID_ECU_BRAKE_PRESSURES:
	  	    	  current_pressure = (*(uint16_t*)buffer)/200.0f;
	  	    	  break;

	  	      case MMR_CAN_MESSAGE_ID_BRK_TARGET_PRESSURE:
	  	        // target_pressure = (((*(uint16_t*)buffer)-1000)/9000.0f) * max_pressure; // ECU, EMA
	  	    	target_pressure=(*(float*)buffer) * max_pressure;
	  	        break;

	  	      case MMR_CAN_MESSAGE_ID_BRK_PROPORTIONAL_ERROR_LEFT_X:
	  	    	  p_data_pressure_error.left_x=(*(float*)buffer);
	  	    	  break;

	  	      case MMR_CAN_MESSAGE_ID_BRK_PROPORTIONAL_ERROR_RIGHT_X:
	  	    	  p_data_pressure_error.right_x=(*(float*)buffer);
	  	    	  break;

	  	      case MMR_CAN_MESSAGE_ID_BRK_PROPORTIONAL_ERROR_LEFT_Y:
	  	    	  p_data_pressure_error.left_y=(*(uint32_t*)buffer);
	  	    	  break;

	  	      case MMR_CAN_MESSAGE_ID_BRK_PROPORTIONAL_ERROR_RIGHT_Y:
	  	    	  p_data_pressure_error.right_y=(*(uint32_t*)buffer);
	  	    	  break;

	  	      case MMR_CAN_MESSAGE_ID_BRK_RISE_CUTOFF_FREQUENCY:
	  	    	  lowpass32_data.cutoff_frequency=(*(float*)buffer);
	  	    	  break;

	  	      case MMR_CAN_MESSAGE_ID_BRK_ZERO_PRESSURE_AUTOSET:
	  	      	  Autoset();
	  	      	  break;

		  	  case MMR_CAN_MESSAGE_ID_BRK_MAX_PRESSURE:
		  	   	  max_pressure=(*(float*)buffer);
		  	   	  break;

		  	  case MMR_CAN_MESSAGE_ID_BRK_CHECK_ASB_STATE:
		  		  target_pressure=max_pressure/2;
		  		  mode = CHECK_ASB;
		  		  break;
	  	      }
	  	    }


	  	    if(target_pressure>max_pressure){
	  	    	target_pressure=max_pressure;
	  	    } else if(target_pressure < 0.0f){
	  	    	target_pressure = 0.0f;
	  	    }

	  	    /*
	  	    if(current_angle>120){
	  	    	target_angle=-140;
	  	    } else if (current_angle < -120){
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV8;
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
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 2000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 100-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
