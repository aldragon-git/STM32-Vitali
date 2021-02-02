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


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "scheduler.h"
/* Printf includes. */
#include "stdio.h"
#include "retarget.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void vSchedulerBlockTrace(void);
void vSchedulerSuspendTrace(TaskHandle_t xTaskHandle);
void vSchedulerReadyTrace( TaskHandle_t xTaskHandle );
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
TaskHandle_t xHandle1 = NULL;
TaskHandle_t xHandle2 = NULL;
TaskHandle_t xHandle3 = NULL;
TaskHandle_t xHandle4 = NULL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void testFuncA1( void *pvParameters );
static void testFuncA2( void *pvParameters );
static void testFuncA3( void *pvParameters );
static void testFuncA4( void *pvParameters );

static void testFuncS1( void *pvParameters );
static void testFuncS2( void *pvParameters );
static void testFuncS3( void *pvParameters );
static void testFuncS4( void *pvParameters );

static void testFunc1( void *pvParameters );
static void testFunc2( void *pvParameters );
static void testFunc3( void *pvParameters );
static void testFunc4( void *pvParameters );

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
  RetargetInit(&huart2);
  vSchedulerInit();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  printf( " ------- Hello from FreeRtos ----------\n\r" );

  char c1 = 'a';
  char c2 = 'b';
  char c3 = 'c';
  char c4 = 'e';

  vSchedulerPeriodicTaskCreate( testFunc1, "t1", configMINIMAL_STACK_SIZE, &c1, 1, &xHandle1, pdMS_TO_TICKS(0), pdMS_TO_TICKS(400), pdMS_TO_TICKS(100), pdMS_TO_TICKS(400));
  vSchedulerPeriodicTaskCreate( testFunc2, "t2", configMINIMAL_STACK_SIZE, &c2, 2, &xHandle2, pdMS_TO_TICKS(150), pdMS_TO_TICKS(300), pdMS_TO_TICKS(100), pdMS_TO_TICKS(100));
  vSchedulerPeriodicTaskCreate( testFunc3, "t3", configMINIMAL_STACK_SIZE, &c3, 3, &xHandle3, pdMS_TO_TICKS(0), pdMS_TO_TICKS(600), pdMS_TO_TICKS(100), pdMS_TO_TICKS(300));
  vSchedulerPeriodicTaskCreate( testFunc4, "t4", configMINIMAL_STACK_SIZE, &c4, 4, &xHandle4, pdMS_TO_TICKS(0), pdMS_TO_TICKS(600), pdMS_TO_TICKS(100), pdMS_TO_TICKS(100));

  vSchedulerAperiodicJobCreate( testFuncA1, "A1", "A1-1", pdMS_TO_TICKS( 100 ) );
  vSchedulerAperiodicJobCreate( testFuncA2, "A2", "A2-1", pdMS_TO_TICKS( 50 ) );
  vSchedulerAperiodicJobCreate( testFuncA3, "A3", "A3-1", pdMS_TO_TICKS( 50 ) );
  vSchedulerAperiodicJobCreate( testFuncA4, "A4", "A4-1", pdMS_TO_TICKS( 50 ) );

  BaseType_t xReturnValue = xSchedulerSporadicJobCreate( testFuncS1, "S1", "S1-1", pdMS_TO_TICKS( 40 ), pdMS_TO_TICKS( 300 ) );
  if( pdFALSE == xReturnValue )
  {
  	printf("Sporadic job S1 not accepted\r\n");
  }

  xReturnValue = xSchedulerSporadicJobCreate( testFuncS2, "S2", "S2-1", pdMS_TO_TICKS( 20 ), pdMS_TO_TICKS( 2500 ) );
  if( pdFALSE == xReturnValue )
  {
  	printf("Sporadic job S2 not accepted\r\n");
  }

  xReturnValue = xSchedulerSporadicJobCreate( testFuncS3, "S3", "S3-1", pdMS_TO_TICKS( 100 ), pdMS_TO_TICKS( 600 ) );
  if( pdFALSE == xReturnValue )
  {
  	printf("Sporadic job S3 not accepted\r\n");
  }
  xReturnValue = xSchedulerSporadicJobCreate( testFuncS4, "S4", "S4-1", pdMS_TO_TICKS( 90 ), pdMS_TO_TICKS( 5000 ) );
  if( pdFALSE == xReturnValue )
  {
  	printf("Sporadic job S4 not accepted\r\n");
  }

  vSchedulerStart();
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
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
static void testFuncA1( void *pvParameters )
{

	int i;
	for( i=0; i < 5000; i++ );
	taskENTER_CRITICAL();
	printf("TICK %d: Aperiodic 1 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();
}

static void testFuncA2( void *pvParameters)
{

	int i;
	for( i=0; i < 8000; i++ );
	taskENTER_CRITICAL();
	printf("TICK %d: Aperiodic 2 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();
}

static void testFuncA3( void *pvParameters)
{

	int i;
	for( i=0; i < 20000; i++ );
	taskENTER_CRITICAL();
	printf("TICK %d: Aperiodic 3 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();
}

static void testFuncA4( void *pvParameters )
{

	int i;
	for( i=0; i < 15000; i++ );
	taskENTER_CRITICAL();
	printf("TICK %d: Aperiodic 4 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();
}

static void testFuncS1( void *pvParameters )
{

	int i;
	for( i=0; i < 20000; i++ );
	taskENTER_CRITICAL();
	printf("TICK %d: Sporadic 1 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();
}


static void testFuncS2( void *pvParameters )
{

	int i;
	for( i=0; i < 50000; i++ );
	taskENTER_CRITICAL();
	printf("TICK %d: Sporadic 2 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();
}

static void testFuncS3( void *pvParameters )
{

	int i;
	for( i = 0; i < 20000; i++ );
	taskENTER_CRITICAL();
	printf("TICK %d: Sporadic 3 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();
}

static void testFuncS4( void *pvParameters )
{

	int i;
	for( i = 0; i < 10000; i++ );
	taskENTER_CRITICAL();
	printf("TICK %d: Sporadic 4 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();
}



static void testFunc1( void *pvParameters ){

	taskENTER_CRITICAL();
	printf("TICK %d: Periodic 1 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();

	int i;
	for( i = 0; i < 90000; i++ );
}


int stopc = 0;
static void testFunc2( void *pvParameters )
{

	int i;
	for( i = 0; i < 85000; i++ );
	taskENTER_CRITICAL();
	printf("TICK %d: Periodic 2 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();

	if( stopc == 3 )
	{
		vSchedulerAperiodicJobCreate( testFuncA1, "A1", "A1-2", pdMS_TO_TICKS( 100 ) );
	}
	if( stopc == 5 )
	{
		if( pdFALSE == xSchedulerSporadicJobCreate( testFuncS1, "S1", "S1-2", pdMS_TO_TICKS( 100 ), pdMS_TO_TICKS( 1000 ) ) )
		{
			printf("TICK %d: Sporadic job S1 not accepted\r\n", (int) xTaskGetTickCount() );
		}
	}
	stopc++;
}

static void testFunc3( void *pvParameters ){

	taskENTER_CRITICAL();
	printf("TICK %d: Periodic 3 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();
	int i;
	for(i = 0; i < 1000000; i++ );
}

static void testFunc4(void *pvParameters)
{

	int i;
	for(i = 0; i < 90000; i++);
	taskENTER_CRITICAL();
	printf("TICK %d: Periodic 4 Execution ############################# 100%% \n\r", (int) xTaskGetTickCount());
	taskEXIT_CRITICAL();
}


/* USER CODE END 4 */

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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
