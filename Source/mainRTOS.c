/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;

osThreadId ReciveTaskHandle;
osThreadId RuidoTaskHandle;
osThreadId ControlTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
void StartTask1(void const * argument);
void StartRuidoTask(void const * argument);
void StartControlTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//variables

struct Gas{
	uint32_t verde,
			 amarillo;

	uint32_t valor;
	uint16_t pos[2];
	uint32_t time;
	bool out[2];
	bool out_prev[2];
	GPIO_TypeDef *port[2];
	int n;
	ADC_ChannelConfTypeDef ch;

	struct Gas *sig;
}CO, NO, SO;
struct Gas* aux;

uint32_t time = 0,
		 MaximoRuido = 1000;

uint8_t dato = 0;

bool f_parlante,
	 f_SdR;

ADC_ChannelConfTypeDef sdr_canal;

//funcion
uint32_t get_adc_value(ADC_ChannelConfTypeDef *canal);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	//limites
	CO.verde=1600;
	CO.amarillo=2600;

	NO.verde=2800;
	NO.amarillo=4000;

	SO.verde=1400;
	SO.amarillo=2400;

	//creacion de lista cicular
	CO.sig = &NO;
	NO.sig = &SO;
	SO.sig = &CO;

	// time init
	CO.time=0;
	SO.time=0;
	NO.time=0;

	// out_prev init
	CO.out_prev[0]=0;
	CO.out_prev[1]=0;
	NO.out_prev[0]=0;
	NO.out_prev[1]=0;
	SO.out_prev[0]=0;
	SO.out_prev[1]=0;

	// asignar ID
	CO.n = 0;
	NO.n = 1;
	SO.n = 2;

	// configuracion de pines
	CO.port[0] = CO1_GPIO_Port;
	CO.port[1] = CO2_GPIO_Port;
	CO.pos[0] = CO1_Pin;
	CO.pos[1] = CO2_Pin;

	NO.port[0] = NO1_GPIO_Port;
	NO.port[1] = NO2_GPIO_Port;
	NO.pos[0] = NO1_Pin;
	NO.pos[1] = NO2_Pin;

	SO.port[0] = SO1_GPIO_Port;
	SO.port[1] = SO2_GPIO_Port;
	SO.pos[0] = SO1_Pin;
	SO.pos[1] = SO2_Pin;

	//configuracion de pines ADC
	CO.ch.Channel = ADC_CHANNEL_1;
	CO.ch.Rank = ADC_REGULAR_RANK_1;
	CO.ch.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	NO.ch.Channel = ADC_CHANNEL_2;
	NO.ch.Rank = ADC_REGULAR_RANK_1;
	NO.ch.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	SO.ch.Channel = ADC_CHANNEL_3;
	SO.ch.Rank = ADC_REGULAR_RANK_1;
	SO.ch.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	sdr_canal.Channel = ADC_CHANNEL_4;
	sdr_canal.Rank = ADC_REGULAR_RANK_1;
	sdr_canal.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;

	// incializacion de puntero que recorre lista circular
	aux = &CO;

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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  /* definition and creation of ReciveTask */
  osThreadDef(ReciveTask, StartTask1, osPriorityAboveNormal, 0, 128);
  ReciveTaskHandle = osThreadCreate(osThread(ReciveTask), NULL);

  /* definition and creation of RuidoTask */
  osThreadDef(RuidoTask, StartRuidoTask, osPriorityLow, 0, 128);
  RuidoTaskHandle = osThreadCreate(osThread(RuidoTask), NULL);

  /* definition and creation of ControlTask */
  osThreadDef(ControlTask, StartControlTask, osPriorityIdle, 0, 128);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CO1_Pin|CO2_Pin|parlante_Pin|SdR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NO1_Pin|NO2_Pin|SO1_Pin|SO2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CO1_Pin CO2_Pin parlante_Pin SdR_Pin */
  GPIO_InitStruct.Pin = CO1_Pin|CO2_Pin|parlante_Pin|SdR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : NO1_Pin NO2_Pin SO1_Pin SO2_Pin */
  GPIO_InitStruct.Pin = NO1_Pin|NO2_Pin|SO1_Pin|SO2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint32_t get_adc_value(ADC_ChannelConfTypeDef *canal){
	if(HAL_ADC_ConfigChannel(&hadc1, canal) == HAL_ERROR)
		Error_Handler();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 100);
	uint32_t valor = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return valor;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask1 */
void StartTask1(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	aux->valor = get_adc_value(&aux->ch);
	  if(aux->valor < aux->verde){
		  // salida
		  aux->out[0] = RESET;
		  aux->out[1] = RESET;
		  // generar dato para transmicion
		  dato = dato & ~(1 << aux->n*2);
		  dato = dato & ~(1 << (aux->n*2+1));
	  }
	  else if(aux->valor < aux->amarillo){
		  aux->out[0] = RESET;
		  aux->out[1] = SET;

		  dato = dato & ~(1 << aux->n*2);
		  dato = dato | (1 << (aux->n*2+1));
	  }
	  else{
		  aux->out[0] = SET;
		  aux->out[1] = SET;
		  HAL_GPIO_WritePin(parlante_GPIO_Port, parlante_Pin, RESET);

		  dato = dato | (1 << aux->n*2);
		  dato = dato & ~(1 << (aux->n*2+1));
	  }
	if(aux==&SO) dato |= (1 << 8);
	aux = aux->sig;
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartRuidoTask */
/**
* @brief Function implementing the RuidoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRuidoTask */
void StartRuidoTask(void const * argument)
{
  /* USER CODE BEGIN StartRuidoTask */
  /* Infinite loop */
  for(;;)
  {
	  if(dato && (1 << 8)){
		  if(get_adc_value(&sdr_canal) > MaximoRuido){
			  f_SdR = SET;
			  f_parlante = SET;
			  dato |= (0 << 6);
		  }else{
			  f_SdR = RESET;
			  f_parlante = RESET;
			  dato &= ~(1 << 6);
		  }
		  HAL_UART_Transmit(&huart1, &dato, 1, 0xffff);
		  dato &= ~(1 << 8);
	  }
	  osDelay(3);
  }
  /* USER CODE END StartRuidoTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void const * argument)
{
  /* USER CODE BEGIN StartControlTask */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(CO.port[0], CO.pos[0], CO.out[0]);
	  HAL_GPIO_WritePin(CO.port[1], CO.pos[1], CO.out[1]);

	  HAL_GPIO_WritePin(NO.port[0], NO.pos[0], NO.out[0]);
	  HAL_GPIO_WritePin(NO.port[1], NO.pos[1], NO.out[1]);

	  HAL_GPIO_WritePin(SO.port[0], SO.pos[0], SO.out[0]);
	  HAL_GPIO_WritePin(SO.port[1], SO.pos[1], SO.out[1]);

	  HAL_GPIO_WritePin(parlante_GPIO_Port, parlante_Pin, !f_parlante);
	  HAL_GPIO_WritePin(SdR_GPIO_Port, SdR_Pin, f_SdR);

	  osDelay(2000);
  }
  /* USER CODE END StartControlTask */
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
	while (1) {
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

