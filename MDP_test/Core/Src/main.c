/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for show_task */
osThreadId_t show_taskHandle;
const osThreadAttr_t show_task_attributes = {
  .name = "show_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Motor */
osThreadId_t MotorHandle;
const osThreadAttr_t Motor_attributes = {
  .name = "Motor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderTask */
osThreadId_t EncoderTaskHandle;
const osThreadAttr_t EncoderTask_attributes = {
  .name = "EncoderTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Servo */
osThreadId_t ServoHandle;
const osThreadAttr_t Servo_attributes = {
  .name = "Servo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void show(void *argument);
void motor(void *argument);
void Encoder(void *argument);
void servo(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer[10];
char choice;
int i = 0; //for switch case in motor
int sep_index = 0;
int totalA = 0; //for pulses in encoder
int totalB = 0;
//float dist; //in cm(used in motor)
int angle; //in degrees(used in motor)

int cntA, cntB;



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
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();

  HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer,10);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of show_task */
  show_taskHandle = osThreadNew(show, NULL, &show_task_attributes);

  /* creation of Motor */
  MotorHandle = osThreadNew(motor, NULL, &Motor_attributes);

  /* creation of EncoderTask */
  EncoderTaskHandle = osThreadNew(Encoder, NULL, &EncoderTask_attributes);

  /* creation of Servo */
  ServoHandle = osThreadNew(servo, NULL, &Servo_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RES_Pin|OLED_DC_Pin
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RES_Pin OLED_DC_Pin
                           LED3_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RES_Pin|OLED_DC_Pin
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//prevent unused argument(s) compilation warning
	UNUSED(huart);

	//put what to transmit into buffer
	//HAL_UART_Transmit(&huart3,(uint8_t *) aRxBuffer,10,0xFFFF);
	HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer,10);
	i=0;
	//memset(aRxBuffer, 0, sizeof(aRxBuffer)); // Reset array

}

/* USER CODE END 4 */
char* substring(char *destination, const char *source, int beg, int n){
	//extract n characters from source string starting from beg index
	//copy into destination string

	while(n>0){
		*destination = *(source+beg);
		destination++;
		source++;
		n--;
	}
	//null termaination destination string
	*destination = '\0';
	return destination;
}

void driveDistance(float distance, uint16_t A, uint16_t B, int direction){
	int offset = 3;
	uint16_t pwmValA = A;
	uint16_t pwmValB = B;
	uint32_t tick;


	long leftcount = 0;
	long rightcount = 0;
	long currentcount = 0;
	long prevleftcount = 0;
	long prevrightcount = 0;
	long leftdiff, rightdiff;

//	float correction = -2.0;
//	distance = distance + correction;

	float countsPerRev = 1290; //cntA value per wheel revolution
	float wheelDiam = 6.43;
	float wheelCirc = PI * wheelDiam;

	float numRev = distance/wheelCirc;
	float targetcount = numRev * countsPerRev;

	//reset counter values
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SET_COUNTER(&htim3,0);
	osDelay(100);

	if(direction == 1){ //forward
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET); //AIN2 ON
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET); //AIN1 OFF

		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET); //AIN1 ON
		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET); //AIN2 OFF

		// Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
	}
	else if(direction == 0){
		HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET); //AIN2 OFF
		HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET); //AIN1 ON

		HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET); //AIN1 OFF
		HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET); //AIN2 ON

		// Modify the comparison value for the duty cycle
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
	}


	 tick = HAL_GetTick(); //Provides a tick value in millisecond

	 if(direction == 1){ //if forward use leftcount
		 currentcount = leftcount;
	 }
	 else if(direction == 0){ //if backward use rightcount
		 currentcount = rightcount;
	 }

	 while(currentcount<targetcount){
		 if(HAL_GetTick()-tick > 100L){
			 leftcount = __HAL_TIM_GET_COUNTER(&htim2);
			 rightcount = __HAL_TIM_GET_COUNTER(&htim3);

			 leftdiff = abs(leftcount - prevleftcount);
			 rightdiff = abs(rightcount - prevrightcount);

			 prevleftcount = leftcount;
			 prevrightcount = rightcount;

			 if(leftdiff>rightdiff){
				 pwmValA = pwmValA - offset;
				 pwmValB = pwmValB + offset;
			 }
			 else if(leftdiff<rightdiff){
				 pwmValA = pwmValA + offset;
				 pwmValB = pwmValB - offset;
			 }
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
			 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);

			 if(direction == 1){ //if forward use leftcount
				 currentcount = leftcount;
			 }
			 else if(direction == 0){ //if backward use rightcount
				 currentcount = rightcount;
			 }
			 tick = HAL_GetTick();
		 }
	 }
	 pwmValA = 0;
	 pwmValB = 0;
	 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
	 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
}

void turnAngle(float degree, int direction){ //direction 0 for left, 1 for right
	uint32_t tick;
	 uint16_t pwmValA;
	 uint16_t pwmValB;

	long leftcount = 0;
	long rightcount = 0;
	long righttemp = 0;

	float countsPerRev = 1290; //cntA value per wheel revolution
	float wheelDiam = 6.43;
	float wheelCirc = PI * wheelDiam;
//	float turningRad = 12.75 ;
//	float correction = 0;

//	if(direction == 1){
//		correction = 1.80;
//	}
//	else if(direction == 0){
//		correction = 7;
//	}
	float distance;

	//float distance = (2 * PI * turningRad * (degree/360.0)) + correction;
	if(direction == 0){ //left
		distance = 33.5;
	}
	else if(direction == 1){ //right
		distance = 32.75;
	}
	float numRev = distance/wheelCirc;
	float targetcount = numRev * countsPerRev;

	//reset counter values
	__HAL_TIM_SET_COUNTER(&htim2,0);
	__HAL_TIM_SET_COUNTER(&htim3,0);
	osDelay(100);

	 HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET); //AIN2 ON
	 HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET); //AIN1 OFF

	 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET); //AIN1 ON
	 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET); //AIN2 OFF

	 tick = HAL_GetTick(); //Provides a tick value in millisecond

	 if(direction == 0){ //left turn
		 pwmValA = 1500; //500
		 pwmValB = 1500; //1500

		 // Modify the comparison value for the duty cycle
		 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
		 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
		 while(leftcount<targetcount){ //count inner wheel distance travelled
			 if(HAL_GetTick()-tick > 100L){
				 leftcount = __HAL_TIM_GET_COUNTER(&htim2);
				 tick = HAL_GetTick();
			 }
		 }
	 }
	 else if(direction == 1){ //right turn
		 pwmValA = 1500;
		 pwmValB = 1500;

		 // Modify the comparison value for the duty cycle
		 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
		 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
		 while(rightcount<targetcount){
			 if(HAL_GetTick()-tick > 100L){
				 righttemp = __HAL_TIM_GET_COUNTER(&htim3);
				 rightcount = 65535 - righttemp;
				 tick = HAL_GetTick();
				 }
			 }
	 }
	 pwmValA = 0;
	 pwmValB = 0;
	 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
	 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
//	  uint8_t ch = 'A';
	  for(;;)
	  {
//		  HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);
//		  if(ch<'Z')
//			  ch++;
//		  else ch = 'A';
		  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	      osDelay(5000);
	  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_show */
/**
* @brief Function implementing the show_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_show */
void show(void *argument)
{
  /* USER CODE BEGIN show */
	uint8_t hello[20] = "hello\0";
  /* Infinite loop */
  for(;;)
  {
	sprintf(hello,"%s\0",aRxBuffer);
	OLED_ShowString(10,10,hello); //write hello in ram of oled
	OLED_Refresh_Gram(); //refresh ram to show string
    osDelay(1000); //refresh every 1s
  }
  /* USER CODE END show */
}

/* USER CODE BEGIN Header_motor */
/**
* @brief Function implementing the Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor */
void motor(void *argument)
{
  /* USER CODE BEGIN motor */
  /* Infinite loop */
	  uint16_t pwmVal = 0;
	  uint16_t pwmValA = 0;
	  uint16_t pwmValB = 0;
	  choice = aRxBuffer[i];
	  uint8_t done[] = "done";
	  float dist;

	  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1); //start motorA
	  HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2); //start motorB
//	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4); //start servor motor

	  /* Infinite loop */
	  for(;;)
	  {
//		  if(sep_index>40){
//			  memset(aRxBuffer, 0, sizeof(aRxBuffer)); // Reset array
//			  sep_index = 0;
//			  i = 0;
//		  }
		  //i=0;
		  //HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer,4);

		  for (int ii=0; ii<10; ii++){
			  if (aRxBuffer[i+ii] == '|') {
				  sep_index = i+ii;
			  }
		  }
		  //sep_index=i+1;
		  if (sep_index <= i) {
			  continue;
		  }

		  choice = aRxBuffer[i];

		  char temp[10];
		  substring(temp,aRxBuffer,i+1,sep_index-i-1);

		  dist = atof(temp);
		  //dist = 10;

		  switch(choice)
		  {
		  	  case 'w': //forward by dist
		  		  //HAL_UART_Transmit(&huart3,(uint8_t *)&done,1,0xFFFF);
		  		  driveDistance(dist,1500,1500,1);
		  		  i = sep_index + 1;
		  		  HAL_UART_Transmit(&huart3,(uint8_t *)&done,5,0xFFFF);
		  		  break;

		  	  case 'f': //forward
		  		  pwmValA=800;
				  pwmValB=800;
				  //AIN2_Pin|AIN1_Pin;
				  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET); //AIN2 ON
				  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET); //AIN1 OFF

				  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET); //AIN1 ON
				  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET); //AIN2 OFF

				  // Modify the comparison value for the duty cycle
				  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
				  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);

		  		  i = sep_index + 1;
				  osDelay(10); //comment out on forward assessment
				  break;

		  	  case 's': //backward by dist
		  		  //HAL_UART_Transmit(&huart3,(uint8_t *)&done,1,0xFFFF);
		  		  driveDistance(dist,1500,1500,0);
		  		  i = sep_index + 1;
		  		  HAL_UART_Transmit(&huart3,(uint8_t *)&done,5,0xFFFF);
		  		  break;

		  	  case 'b': //backward
		  		  pwmValA=800;
		  		  pwmValB=800;
		  		  HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET); //AIN2 OFF
		  		  HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET); //AIN1 ON

		  		  HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET); //AIN1 OFF
		  		  HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET); //AIN2 ON
		  		  pwmVal++;
		  		  // Modify the comparison value for the duty cycle
		  		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
		  		  __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);

		  		  osDelay(10);

//		  		  pwmVal = 0;
//					 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmVal);
//					 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmVal);

//		  		  i++;
		  		  i = sep_index + 1;
		  		  break;

		  	  case 'x':
		  		  pwmValA = 0;
		  		  pwmValB = 0;
//		  			HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET); //AIN2 ON
//		  			HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET); //AIN1 OFF
//
//					 HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET); //AIN1 ON
//					 HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET); //AIN2 OFF

		  			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
		  			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
		  			//osDelay(10);

//					 i++;
					 i = sep_index + 1;
					 break;

		  	  case 'a': //turn left 90
		  		  //
		  		  htim1.Instance->CCR4 = 54;   //extreme left
		  		  osDelay(10);

		  		  turnAngle(90,0);

		  		  htim1.Instance->CCR4 = 81; //right
		  		  osDelay(500);
		  		  htim1.Instance->CCR4 = 73; //center
		  		  osDelay(10);

		  		  i = sep_index + 1;
		  		  HAL_UART_Transmit(&huart3,(uint8_t *)&done,5,0xFFFF);
		  		  break;

		  	  case 'd': //turn right 90
		  		  //
		  		  htim1.Instance->CCR4 = 98; //extreme right
		  		  osDelay(10);

		  		  turnAngle(90,1);

		  		  htim1.Instance->CCR4 = 81; //right
		  		  osDelay(500);
		  		  htim1.Instance->CCR4 = 73; //center
		  		  osDelay(10);

		  		  i = sep_index + 1;
		  		  HAL_UART_Transmit(&huart3,(uint8_t *)&done,5,0xFFFF);
		  		  break;

		  	  case'r':
					htim1.Instance->CCR4 = 98; //extreme right 98
					osDelay(10);
			  		i = sep_index + 1;
					//choice = aRxBuffer[i];
					break;

		  	  case 'l':
					htim1.Instance->CCR4 = 54;   //extreme left 54
					osDelay(10);
					i = sep_index + 1;
					//choice = aRxBuffer[i];
					break;

		  	  case 'c':
		  			htim1.Instance->CCR4 = 81; //right
		  			osDelay(500);
		  			htim1.Instance->CCR4 = 73; //center
		  			osDelay(10);
		  			i = sep_index + 1;
					//choice = aRxBuffer[i];
					break;
//		  	  case 'g':  //aQCcWCbBC
//					pwmValA = 800;
//					pwmValB = 825;
//					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_SET);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
//					osDelay(3000);
//
//					pwmValA = pwmValB = 0;
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
//					osDelay(1000);
//
//					htim1.Instance->CCR4 = 94;
//					osDelay(1000);
//
//					pwmValA = 800;
//					pwmValB = 825;
//					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
//					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
//					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
//					osDelay(7800);
//
//					pwmValA = pwmValB = 0;
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
//					osDelay(1000);
//
//					htim1.Instance->CCR4 = 51;
//					osDelay(1000);
//
//					pwmValA = 770; //800
//					pwmValB = 875; //825
//					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
//					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
//					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
//					osDelay(11500);
//
//					pwmValA = pwmValB = 0;
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
//					osDelay(500);
//
//					htim1.Instance->CCR4 = 81;
//					osDelay(250);
//					htim1.Instance->CCR4 = 73;
//
//					pwmValA = 800;
//					pwmValB = 825; //825
//					HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
//					HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
//					HAL_GPIO_WritePin(GPIOA, BIN2_Pin, GPIO_PIN_SET);
//					HAL_GPIO_WritePin(GPIOA, BIN1_Pin, GPIO_PIN_RESET);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
//					osDelay(1000);
//
//					pwmValA = pwmValB = 0;
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
//					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, pwmValB);
//
//
//					i = sep_index + 1;
//					HAL_UART_Transmit(&huart3,(uint8_t *)&ch,1,0xFFFF);
//					break;
//
				  default:
					  //i++;
					 // choice = aRxBuffer[i];
					  break;
		  }

	  }

  /* USER CODE END motor */
}

/* USER CODE BEGIN Header_Encoder */
/**
* @brief Function implementing the EncoderTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Encoder */
void Encoder(void *argument)
{
	  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL); //motorA
	  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL); //motorB

	  cntA = __HAL_TIM_GET_COUNTER(&htim2);
	  cntB = __HAL_TIM_GET_COUNTER(&htim3);
//
//	  int cnt1A, cnt2A, cnt1B, cnt2B; //timer counts
//	  int diffA, diffB; //number of pulses per every 1000 ticks (speed)
//	  int offset = 5;
	  uint32_t tick;
//
//	  cnt1A = __HAL_TIM_GET_COUNTER(&htim2); //Get the TIM Counter Register value on runtime.
//	  cnt1B = __HAL_TIM_GET_COUNTER(&htim3); //Get the TIM Counter Register value on runtime.
//
	  tick = HAL_GetTick(); //Provides a tick value in millisecond
	  uint8_t hello[20];
	  uint16_t dirA;
	  uint16_t dirB;

	  for(;;)
	  {
//		//trailing l makes the type of the constant
//		//a long int instead of a regular int
		if(HAL_GetTick()-tick > 250L){ //delay 250 ticks
			cntA = __HAL_TIM_GET_COUNTER(&htim2);
			cntB = __HAL_TIM_GET_COUNTER(&htim3);

			if(cntB != 0){
				cntB = 65535 - cntB;
			}
//			cnt2A = __HAL_TIM_GET_COUNTER(&htim2);
//			cnt2B = __HAL_TIM_GET_COUNTER(&htim3);
//
//			//False (Counter used as upcounter) or True (Counter used as downcounter)
//			//HTIM2
//			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)){ //true
//				if(cnt2A<=cnt1A)
//					diffA = cnt1A - cnt2A; //both cnt1 and cnt2 within downcounter value
//											//cnt1 should be bigger
//				else
//					diffA = (65535 - cnt2A)+cnt1A; //counter was counting up 1000 ticks ago(cnt2>cnt1)
//			}
//			else{ //false
//				if(cnt2A >= cnt1A)
//					diffA = cnt2A - cnt1A;
//				else
//					diffA = (65535 - cnt1A) + cnt2A;
//			}
//
//			//HTIM3
//			if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3)){ //true
//				if(cnt2B<=cnt1B)
//					diffB = cnt1B - cnt2B;
//				else
//					//diffB = (65535 - cnt2B)+cnt1B;
//					diffB = (65535 - cnt2B)+cnt1B;
//			}
//			else{ //false
//				if(cnt2B >= cnt1B)
//					diffB = (cnt2B - cnt1B);
//				else
//					diffB = ((65535 - cnt1B) + cnt2B);
//			}
//
//
/////////////////////CALCULATING TOTAL PULSES///////////////////////////////////////////////////////
//			if(diffA >= 65535){
//				totalA = totalA + diffA - 65535;
//			}
//			else{
//				totalA = totalA + diffA; //accumulate no. of pulses for A
//			}
//
//			if(diffB >= 65535){
//				totalB = totalB + diffB - 65535;
//			}
//			else{
//				totalB = totalB + diffB; //accumulate no. of pulses for B
//			}
//
////////////////////FOR STOPPING AFTER A CERTAIN DISTANCE/////////////////////////////////////////////////
			//get number of pulses needed
//			int temp = dist/10*600; //600 pulses is ard 10cm
//			if(totalA>temp){ //stop if number of pulses for A reaches temp
//				 pwmValA=0;
//				 pwmValB=0;
//				 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
//				 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
//				 osDelay(10);
//				 totalA = 0;
//				 totalB = 0;
//			 }

///////////////////FOR MOVING STRAIGHT WHILE DISTANCE IS NOT REACHED////////////////////////////
			//while robot not stopping yet AND is moving
//			else{
//				 if(pwmValA>0){
//					 if(diffA > diffB){ //increase in number of pulses for A is more than B(after 1000 ticks)
//						 pwmValA = pwmValA - offset; //slow down
//						 pwmValB = pwmValB + offset; //speed up
//					 }
//					 else if(diffA < diffB){ //increase in number of pulses for B>A
//						 pwmValA = pwmValA + offset;
//						 pwmValB = pwmValB - offset;
//
//					 }
//					 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,pwmValA);
//					 __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_2,pwmValB);
//					 osDelay(10);
//				 }
//			 }

			//display total pulses of A and B
			sprintf(hello,"cntA:%5d\0",cntA);
			OLED_ShowString(10,20,hello);

			sprintf(hello,"cntB:%5d\0",cntB);
			OLED_ShowString(10,30,hello);

			dirA = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
			dirB = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);

			sprintf(hello,"DirA:%5d\0",dirA);
			OLED_ShowString(10,40,hello);

			sprintf(hello,"DirB:%5d\0",dirB);
			OLED_ShowString(10,50,hello);

//			cnt1A = __HAL_TIM_GET_COUNTER(&htim2);
//			cnt1B = __HAL_TIM_GET_COUNTER(&htim3);

			tick = HAL_GetTick();
		}
	   // osDelay(1);
	  }
	  /* USER CODE END encoder_task */
  /* USER CODE END Encoder */
}

/* USER CODE BEGIN Header_servo */
/**
* @brief Function implementing the Servo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo */
void servo(void *argument)
{
  /* USER CODE BEGIN servo */
  /* Infinite loop */
	  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4); //start servor motor
	  choice = aRxBuffer[i];
	for(;;)
  {
		  //HAL_UART_Receive_IT(&huart3,(uint8_t *) aRxBuffer,10);
		  choice = aRxBuffer[i];
	  switch(choice)
	  {
//	  	  case'r':
//				htim1.Instance->CCR4 = 98; //extreme right
//				osDelay(10);
//		  		i = sep_index + 1;
//				//choice = aRxBuffer[i];
//				break;
//
//	  	  case 'l':
//				htim1.Instance->CCR4 = 54;   //extreme left
//				osDelay(10);
//				i = sep_index + 1;
//				//choice = aRxBuffer[i];
//				break;
//	  	  case 'c':
//	  			htim1.Instance->CCR4 = 81; //right
//	  			osDelay(500);
//	  			htim1.Instance->CCR4 = 73; //center
//	  			osDelay(10);
//	  			i = sep_index + 1;
//				//choice = aRxBuffer[i];
//				break;

	  	  //default:
	  		  //i++;
//	  		  htim1.Instance->CCR4 = 84; //right
//	  		  htim1.Instance->CCR4 = 74; //center
	  		  //osDelay(10);
//	  		  i++;
			  //choice = aRxBuffer[i];
	  		 // break;
	  }
    //osDelay(1);
  }

//	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
//	for(;;)
//	{
//		htim1.Instance->CCR4 = 85;   //extreme right
//		osDelay(5000);
//		htim1.Instance->CCR4 = 74;    //center
//		osDelay(5000);
//		htim1.Instance->CCR4 = 60;    //extreme left
//		osDelay(5000);
//		htim1.Instance->CCR4 = 74;    //center
//		osDelay(5000);
//	}
  /* USER CODE END servo */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
