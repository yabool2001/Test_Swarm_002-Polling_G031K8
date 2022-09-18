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
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SWARM_UART_TX_TIMEOUT		500
#define SWARM_UART_RX_TIMEOUT		250
#define SWARM_UART_RX_XL_TIMEOUT	5000
#define RX_BUFF_SIZE				200
#define TX_BUFF_SIZE				250
#define PW_BUFF_SIZE				5
#define GN_BUFF_SIZE				35
#define TD_PAYLOAD_BUFF_SIZE		90
#define AT_SWARM_BUFF_SIZE			100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t				checklist				= 0 ;
uint8_t				tim14_on				= 0 ;
uint8_t				tim16_on	 			= 0 ;

HAL_StatusTypeDef   uart_status ;
uint8_t             rx_buff[RX_BUFF_SIZE] ;
uint8_t             tx_buff[TX_BUFF_SIZE] ;
char				pw_buff[PW_BUFF_SIZE] ;
char				gn_buff[GN_BUFF_SIZE] ;
char 				td_at_comm[TD_PAYLOAD_BUFF_SIZE] ;
char* 				chunk ;
uint16_t			rx_len =  RX_BUFF_SIZE ;// stosowane tylko HAL_UARTEx_ReceiveToIdle()

// SWARM AT Commands
const char*			cs_at_comm				= "$CS\0" ;
const char*			rt_0_at_comm			= "$RT 0\0" ;
const char*			rt_q_rate_at_comm		= "$RT ?\0" ;
const char*			pw_0_at_comm			= "$PW 0\0" ;
const char*			pw_q_rate_at_comm		= "$PW ?\0" ;
const char*			pw_mostrecent_at_comm	= "$PW @\0" ;
const char*			dt_0_at_comm			= "$DT 0\0" ;
const char*			dt_q_rate_at_comm		= "$DT ?\0" ;
const char*			gs_0_at_comm			= "$GS 0\0" ;
const char*			gs_q_rate_at_comm		= "$GS ?\0" ;
const char*			gj_0_at_comm			= "$GJ 0\0" ;
const char*			gj_q_rate_at_comm		= "$GJ ?\0" ;
const char*			gn_0_at_comm			= "$GN 0\0" ;
const char*			gn_q_rate_at_comm		= "$GN ?\0" ;
const char*			gn_mostrecent_at_comm	= "$GN @\0" ;
const char*			dt_mostrecent_at_comm	= "$DT @\0" ;
const char*			mt_del_all_at_comm		= "$MT D=U\0" ;
const char 			td_mzo_at_comm[]		= "$TD HD=60,\"MZO\"" ; // to ładnie działało i przeszło przez satelitę
const char*			sl_3ks_at_comm			= "$SL S=3000\0" ; // 50 minut spania dla Swarm
const char*			sl_3c5ks_at_comm		= "$SL S=3500\0" ; // 50-2 minut spania dla Swarm
const char*			sl_3c4ks_at_comm		= "$SL S=3400\0" ; // 60-3 minut spania dla Swarm
const char*			sl_60s_at_comm			= "$SL S=60\0" ; // 60s spania dla Swarm
const char*			sl_120s_at_comm			= "$SL S=120\0" ; // 120s spania dla Swarm
const char*			sl_5s_at_comm			= "$SL S=5\0" ; // 120s spania dla Swarm

// SWARM AT Answers
const char*         cs_answer				= "$CS DI=0x\0" ;
const char*         rt_ok_answer			= "$RT OK*22\0" ;
const char*         rt_0_answer				= "$RT 0*16\0" ;
const char*         pw_ok_answer			= "$PW OK*23\0" ;
const char*         pw_0_answer				= "$PW 0*17\0" ;
const char*         pw_mostrecent_answer	= "$PW \0" ;
const char*         dt_ok_answer			= "$DT OK*34\0" ;
const char*         dt_0_answer				= "$DT 0*00\0" ;
const char*         gs_ok_answer			= "$GS OK*30\0" ;
const char*         gs_0_answer				= "$GS 0*04\0" ;
const char*         gj_ok_answer			= "$GJ OK*29\0" ;
const char*         gj_0_answer				= "$GJ 0*1d\0" ;
const char*         gn_ok_answer			= "$GN OK*2d\0" ;
const char*         gn_0_answer				= "$GN 0*19\0" ;
const char*         gn_mostrecent_answer	= "$GN \0" ;
const char*			mt_del_all_answer		= "$MT \0" ; // nie wiadomo ile ich będzie dlatego nie mogę ustawić "$MT 0*09"
const char*			td_ok_answer			= "$TD OK,\0" ;
const char*         sl_ok_answer			= "$SL OK*3b\0" ;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
void send2swarm_at_command ( const char* , const char* , uint8_t ) ;
void pw2payload ( void ) ;
void gn2payload ( void ) ;
uint8_t nmea_checksum ( const char* , size_t ) ;
void wait_for_tim16x ( uint8_t ) ;
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
  MX_RTC_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
  __HAL_TIM_CLEAR_IT ( &htim14 , TIM_IT_UPDATE ) ; // żeby nie generować przerwania TIM6 od razu: https://stackoverflow.com/questions/71099885/why-hal-tim-periodelapsedcallback-gets-called-immediately-after-hal-tim-base-sta
  __HAL_TIM_CLEAR_IT ( &htim16 , TIM_IT_UPDATE ) ; // żeby nie generować przerwania TIM6 od razu: https://stackoverflow.com/questions/71099885/why-hal-tim-periodelapsedcallback-gets-called-immediately-after-hal-tim-base-sta
  //wait_for_tim16x ( 2 ) ;

  send2swarm_at_command ( cs_at_comm , cs_answer , 1 ) ;
  if ( checklist == 1 )
	  send2swarm_at_command ( rt_0_at_comm , rt_ok_answer , 2 ) ;
  if ( checklist == 2 )
	  send2swarm_at_command ( rt_q_rate_at_comm , rt_0_answer , 3 ) ; // Query RT rate
  if ( checklist == 3 )
	  send2swarm_at_command ( pw_0_at_comm , pw_ok_answer , 4 ) ;
  if ( checklist == 4 )
	  send2swarm_at_command ( pw_q_rate_at_comm , pw_0_answer , 5 ) ;
  if ( checklist == 5 )
	  send2swarm_at_command ( dt_0_at_comm , dt_ok_answer , 6 ) ;
  if ( checklist == 6 )
	  send2swarm_at_command ( dt_q_rate_at_comm , dt_0_answer , 7 ) ;
  if ( checklist == 7 )
	  send2swarm_at_command ( gs_0_at_comm , gs_ok_answer , 8 ) ;
  if ( checklist == 8 )
	  send2swarm_at_command ( gs_q_rate_at_comm , gs_0_answer , 9 ) ;
  if ( checklist == 9 )
	  send2swarm_at_command ( gj_0_at_comm , gj_ok_answer , 10 ) ;
  if ( checklist == 10 )
	  send2swarm_at_command ( gj_q_rate_at_comm , gj_0_answer , 11 ) ;
  if ( checklist == 11 )
	  send2swarm_at_command ( gn_0_at_comm , gn_ok_answer , 12 ) ;
  if ( checklist == 12 )
	  send2swarm_at_command ( gn_q_rate_at_comm , gn_0_answer , 13 ) ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if ( checklist == 13 )
		  send2swarm_at_command ( pw_mostrecent_at_comm , pw_mostrecent_answer , 14 ) ;
	  //wait_for_tim16x ( 6 ) ;
	  if ( checklist == 14 )
		  send2swarm_at_command ( gn_mostrecent_at_comm , gn_mostrecent_answer , 15 ) ;
	  if ( checklist == 15 )
		  send2swarm_at_command ( mt_del_all_at_comm , mt_del_all_answer , 16 ) ;
	  if ( checklist == 16 )
	  {
		  snprintf ( td_at_comm , TD_PAYLOAD_BUFF_SIZE , "$TD HD=60,\"%s;%s\"" , pw_buff , gn_buff ) ;
		  send2swarm_at_command ( td_at_comm , td_ok_answer , 17 ) ;
		  //send2swarm_at_command ( td_mzo_at_comm , td_ok_answer , 17 ) ;
	  }
	  if ( checklist == 17 )
		  wait_for_tim16x ( 6 ) ;

	  send2swarm_at_command ( sl_3c5ks_at_comm , sl_ok_answer , 18 ) ; // Swarm sleep for 50min.
	  rx_buff[0] = 0 ;
	  pw_buff[0] = 0 ;
	  gn_buff[0] = 0 ;
	  checklist = 13 ;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  /* Enter LowPower Mode */

	  // Option1: Enter the SHUTDOWN mode. App applies hard restart after WFI
	  HAL_PWREx_EnterSHUTDOWNMode () ; // Enter the SHUTDOWN mode. Docelowo rozważyć STOP Mode 2, żeby nie zaczynać zawsze od konfiguracji
	  // Option2: Enter the STOP1. App continues after WFI without restarting variables like chacklist
	  //HAL_PWR_EnterSTOPMode ( PWR_LOWPOWERREGULATOR_ON , PWR_STOPENTRY_WFI ) ;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 60, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 16000-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 2000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 16000-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 10000-1;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GREEN_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void send2swarm_at_command ( const char* at_command , const char* answer , uint8_t step )
{
	//uint32_t temp_tickstart = HAL_GetTick () ; //temp
	uint8_t cs = nmea_checksum ( at_command , strlen ( at_command ) ) ;

	sprintf ( (char*) tx_buff , "%s*%02x\n" , at_command , cs ) ;
	//uart_status = HAL_UART_Receive ( &huart1 , rx_buff , sizeof ( rx_buff ) , SWARM_UART_RX_TIMEOUT ) ;
	uart_status = HAL_UARTEx_ReceiveToIdle ( &huart1 , rx_buff , sizeof ( rx_buff ) , &rx_len , SWARM_UART_RX_TIMEOUT ) ;
	uart_status = HAL_UART_AbortReceive ( &huart1 ) ;
	rx_buff[0] = 0 ;
	uart_status = HAL_UART_Transmit ( &huart1 , (const uint8_t *) tx_buff ,  strlen ( (char*) tx_buff ) , SWARM_UART_TX_TIMEOUT ) ;
	//uart_status = HAL_UART_Receive ( &huart1 , rx_buff , sizeof ( rx_buff ) , SWARM_UART_RX_TIMEOUT ) ;
	uart_status = HAL_UARTEx_ReceiveToIdle ( &huart1 , rx_buff , sizeof ( rx_buff ) , &rx_len , SWARM_UART_RX_TIMEOUT ) ;
	/* Wait of SWARM UARt RX */
	tim16_on = 1 ;
	HAL_TIM_Base_Start_IT ( &htim16 ) ;
	while ( tim16_on )
	{
		if ( strncmp ( (char*) rx_buff , answer , strlen ( answer ) ) == 0 )
		{
			checklist = step ;
			break ;
		}
		else
		{
			rx_buff[0] = 0 ;
			//uart_status = HAL_UART_Receive ( &huart1 , rx_buff , sizeof ( rx_buff ) , SWARM_UART_RX_TIMEOUT ) ;
			uart_status = HAL_UARTEx_ReceiveToIdle ( &huart1 , rx_buff , sizeof ( rx_buff ) , &rx_len , SWARM_UART_RX_TIMEOUT ) ;
		}
	}
	if ( checklist != step && step != 17 )
	{
		uart_status = HAL_UART_Transmit ( &huart1 , (const uint8_t *) tx_buff ,  strlen ( (char*) tx_buff ) , SWARM_UART_TX_TIMEOUT ) ;
		//uart_status = HAL_UART_Receive ( &huart1 , rx_buff , sizeof ( rx_buff ) , SWARM_UART_RX_TIMEOUT ) ;
		uart_status = HAL_UARTEx_ReceiveToIdle ( &huart1 , rx_buff , sizeof ( rx_buff ) , &rx_len , SWARM_UART_RX_TIMEOUT ) ;
		/* Wait of SWARM UARt RX */
		tim16_on = 1 ;
		HAL_TIM_Base_Start_IT ( &htim16 ) ;
		while ( tim16_on )
		{
			//uart_status = HAL_UART_Receive ( &huart1 , rx_buff , sizeof ( rx_buff ) , SWARM_UART_RX_TIMEOUT ) ;
			uart_status = HAL_UARTEx_ReceiveToIdle ( &huart1 , rx_buff , sizeof ( rx_buff ) , &rx_len , SWARM_UART_RX_TIMEOUT ) ;
			if ( strncmp ( (char*) rx_buff , answer , strlen ( answer ) ) == 0 )
			{
				checklist = step ;
				break ;
			}
			else
			{
				rx_buff[0] = 0 ;
				//uart_status = HAL_UART_Receive ( &huart1 , rx_buff , sizeof ( rx_buff ) , SWARM_UART_RX_TIMEOUT ) ;
				uart_status = HAL_UARTEx_ReceiveToIdle ( &huart1 , rx_buff , sizeof ( rx_buff ) , &rx_len , SWARM_UART_RX_TIMEOUT ) ;
			}
		}
	}
	/*
	uart_status = HAL_UART_Receive ( &huart1 , rx_buff , sizeof ( rx_buff ) , SWARM_UART_RX_TIMEOUT ) ;
	if ( strncmp ( (char*) rx_buff , answer , strlen ( answer ) ) == 0 )
		checklist = step ;
	else
	{
		wait_for_tim16x ( 1 ) ;
		uart_status = HAL_UART_Receive ( &huart1 , rx_buff , sizeof ( rx_buff ) , SWARM_UART_RX_XL_TIMEOUT ) ;
		if ( strncmp ( (char*) rx_buff , answer , strlen ( answer ) ) == 0 )
			checklist = step ;
	}
	*/

	if ( strncmp ( pw_mostrecent_at_comm , at_command , strlen ( pw_mostrecent_at_comm ) ) == 0 )
		pw2payload () ;
	if ( strncmp ( gn_mostrecent_at_comm , at_command , strlen ( gn_mostrecent_at_comm ) ) == 0 )
		gn2payload () ;
	rx_buff[0] = 0 ;
}

void pw2payload ()
{
	chunk = strtok ( (char*) rx_buff , " " ) ;
	chunk = strtok ( NULL , "," ) ;
	sprintf ( pw_buff , "%4s" , chunk ) ;
}
void gn2payload ()
{
	chunk = strtok ( (char*) rx_buff , " " ) ;
	chunk = strtok ( NULL , "*" ) ;
	sprintf ( (char*) gn_buff , "%s" , chunk ) ;
}

void wait_for_tim16x ( uint8_t x )
{
	uint8_t i ;
	for ( i = 0 ; i < x ; i++ )
	{
		tim16_on = 1 ;
		HAL_TIM_Base_Start_IT ( &htim16 ) ;
		while ( tim16_on )
			__NOP () ;
	}
}

uint8_t nmea_checksum ( const char *sz , size_t len )
{
	size_t i = 0 ;
	uint8_t cs ;
	if ( sz [0] == '$' )
		i++ ;
	for ( cs = 0 ; ( i < len ) && sz [i] ; i++ )
		cs ^= ( (uint8_t) sz [i] ) ;
	return cs;
}

void HAL_TIM_PeriodElapsedCallback ( TIM_HandleTypeDef *htim )
{
	if ( htim->Instance == TIM14 )
	{
		tim14_on = 0 ;
		HAL_TIM_Base_Stop_IT ( &htim14 ) ;
	}
	if ( htim->Instance == TIM16 )
	{
		tim16_on = 0 ;
		HAL_TIM_Base_Stop_IT ( &htim16 ) ;
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
