/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "joystick.h"
#include "software_timer.h"
#include <stdlib.h>
#include <time.h>
#include "lcd.h"
#include "lcd_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WAIT_KEY_TIMEOUT		5000 //milisecond
#define NUM_OF_MAX_DATA_STORAGE	10

#define SERVICE_22		1
#define SERVICE_27		2
#define SERVICE_2E		3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8] = {0};
uint8_t RxData[8];

uint32_t TxMailbox;

CAN_TxHeaderTypeDef TxHeader2;
CAN_RxHeaderTypeDef RxHeader2;

uint8_t TxData2[8] = {0};
uint8_t RxData2[8];

uint32_t TxMailbox2;

int dataflag = 0;
int dataflag2 = 0;

uint32_t ADCValue = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_IT(&hadc1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

  HAL_TIM_Base_Start_IT(&htim2);

  lcd_init();
	lcd_display_string( 60,60,(uint8_t* )"CPU:STM32F405RGT6          ", FONT_1206, RED );
	lcd_display_string( 60,80,(uint8_t* )"www.WaveShare.net          ", FONT_1206, RED );
	lcd_draw_line(80,150,120,180,BLACK);
	lcd_draw_circle(120,210,20,BLUE);
	lcd_draw_rect(100,250,50,50,GREEN);
	lcd_fill_rect(30,220,50,50,RED);


  TxHeader.DLC = 8;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.StdId = 0x7A2;		//response ID

  TxHeader2.DLC = 8;
  TxHeader2.IDE = CAN_ID_STD;
  TxHeader2.RTR = CAN_RTR_DATA;
  TxHeader2.StdId = 0x712;		//request ID

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  srand(time(NULL)); // Call for initialize random function
  set_timer0(1000);

  //variable for ECU
  uint8_t 	service 		= 0x00;
  uint8_t 	CANFrameType 	= 10;
  uint8_t 	frameDataLength = 0;
  uint16_t 	totalDataLength = 0;
  uint8_t 	CAN_NPCI;
  uint16_t 	dataCount 		= 0;
  uint8_t	dataStorage[NUM_OF_MAX_DATA_STORAGE];
  int 		currentIndex	= 0;
  int		CF_sending_flag = 0; //flag indicate Consecutive Frame sending is not done
  int		s27state 		= 0;

  //variable for TESTER
  int 		joyPressed 		= 0; // flag for any joystick is pressed
  uint8_t	dataToSend[6];
  int 		seedRequested	= 0;
  int		keySent			= 0;

  // service 27 key variable
  uint8_t K12;
  uint8_t K34;
  uint8_t K56;
  uint8_t K78;

  int functionSelection = SERVICE_22;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/* ECU PART */
	if(dataflag){
		dataflag = 0;
		CAN_NPCI = RxData[0]; //Frame type and Data length in CAN_TP
		CANFrameType = CAN_NPCI>>4;
		frameDataLength = CAN_NPCI<<4;
		frameDataLength = frameDataLength >>4;
		// check if single frame or not
		switch (CANFrameType){
		case 0x00:	//single frame
			for(int i = 0; i< frameDataLength; i++){
				dataStorage[i] = RxData[1+i];
			}
			totalDataLength = frameDataLength;
			service = dataStorage[0];
			break;
		case 0x01:	//first frame
			CF_sending_flag = 1;
			totalDataLength = frameDataLength;
			totalDataLength = totalDataLength << 8;
			totalDataLength = totalDataLength + RxData[1];
			dataCount = totalDataLength;
			// store data
			if(dataCount >= 3){
				for(currentIndex = 0; currentIndex<3; currentIndex++){
					dataStorage[currentIndex] = RxData[currentIndex+2];
				}
				dataCount = dataCount-3;
			}else{
				for(int i = 0; i<dataCount; i++){
					dataStorage[currentIndex] = RxData[currentIndex+2];
				}
				dataCount = 0;
			}
			service = dataStorage[0];

			if(dataCount == 0) CF_sending_flag = 0;
			//send flow control
			TxData[0] = 0x30; 	//FT: flow control, Frame State: continue to send
			TxData[1] = 0x08;	//BS: max 8 CFs till next FC
			TxData[2] = 0x19; 	//STmin: 25ms

			TxHeader.DLC = 3;
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			break;
		case 0x02:	// consecutive frame
			if(dataCount >7){
				for(int i = 0; i<7; i++){
					dataStorage[currentIndex++] = RxData[i+1];
				}
				dataCount = dataCount - 7;
			}else{
				for(int i = 0; i<dataCount; i++){
					dataStorage[currentIndex++] = RxData[i+1];
				}
				dataCount = 0;
			}
			if(dataCount == 0) {
				CF_sending_flag = 0;
				break;
			}
			//if Sequential number is max as defined previous resend FC
			if(frameDataLength == 0x08){
				TxData[0] = 0x30; 	//FT: flow control, Frame State: continue to send
				TxData[1] = 0x08;	//BS: max 8 CFs till next FC
				TxData[2] = 0x19; 	//STmin: 25ms

				TxHeader.DLC = 3;
				HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
			}
			break;
		default:
			break;
		}

		// if all CFs is sent, do the function
		if(CF_sending_flag == 0){
			switch (service) {
				case 0x22:
					if(dataStorage[1] == 0xF0 && dataStorage[2] == 0x01){
						TxData[0] = 0x07;	//single frame, data length 7
						TxData[1] = 0x62; 	//positive response code
						TxData[2] = 0xF0;
						TxData[3] = 0x01;
						//convert adc to TxData 4, 5, 6, 7
						TxData[4] = ADCValue>>24;
						TxData[5] = ADCValue>>16;
						TxData[6] = ADCValue>>8;
						TxData[7] = ADCValue;

						TxHeader.DLC = 8;
						HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
					}
					break;
				case 0x2E:
					if(dataStorage[1] == 0xF0 && dataStorage[2] == 0x02){
						TxData[0] = 0x03;	//single frame, data length 3
						TxData[1] = 0x6E; 	//positive response code
						TxData[2] = 0xF0;
						TxData[3] = 0x02;
						TxHeader.DLC = 4;
						HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
						/*todo: display received data to lcd
						 *		(from dataStorage[3] to dataStorage[totalDataLength-1])
						 */
					}
					break;
				case 0x27:
					//check inDelay flag
					//if true, get out of the function
					if(s27delayFlag) break;

					//check subfunction 01: request seed
					if(dataStorage[1] == 0x01)  {
						s27state = 1;
						//if true: gen seed (random algorithm)
						TxData[0] = 0x06;
						TxData[1] = 0x67;
						TxData[2] = 0x01;
						TxData[3] = ((rand()-1) % 0xFF) + 1;
						TxData[4] = ((rand()-1) % 0xFF) + 1;
						TxData[5] = ((rand()-1) % 0xFF) + 1;
						TxData[6] = ((rand()-1) % 0xFF) + 1;
						//send seed
						TxHeader.DLC = 7;
						HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

						//compute key
						K12 = TxData[3] + 1;
						K34 = TxData[4] + 1;
						K56 = TxData[5] + 1;
						K78 = TxData[6] + 1;
					}
					//check subfunction 02: send key
					else if(dataStorage[1]==0x02 && s27state == 1){
						s27state = 2;
						//if true: compare key
						if((RxData[3] == K12 && RxData[4] == K34)
							&& (RxData[5] == K56 && RxData[6] == K78)){
							//if key valid: pos response
							TxData[0] = 0x02;
							TxData[0] = 0x67;
							TxData[1] = 0x02;
							TxHeader.DLC = 3;
							HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);

							//turn on led to indicate unlock state
							HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);

						}else{
							//if key invalid: add delay 10s for 0x27 function
							set_s27delay_timer();
						}
					}
					break;
				default:
					break;
			}
		}
	}

	/* TESTER PART */
	switch (functionSelection) {
		case SERVICE_22:
			//request ADC each second
			if(timer0Flag){
				set_timer0(1000);

				TxData2[0] = 0x03;
				TxData2[1] = 0x22;
				TxData2[2] = 0xF0;
				TxData2[3] = 0x01;

				TxHeader2.DLC = 4;
				HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2);
			}
			if(dataflag2){
				dataflag2 = 0;
				if(RxData2[0]>>4 != 0x00) break;
				if(RxData2[1] != 0x62) break;
				uint32_t receivedAdcValue = 0;
				for(int i = 0; i<4; i++){
					receivedAdcValue = receivedAdcValue << 2;
					receivedAdcValue = receivedAdcValue + RxData2[i+4];
				}
				//todo: display received Value on lcd
			}
			break;
		case SERVICE_2E:
			if(is_joy_pressed(JOY_CTR)){
				joyPressed = 1;
				for(int i = 0; i < 6; i++){
					dataToSend[i]= 0x00;
				}
			}
			if(is_joy_pressed(JOY_LEFT)){
				joyPressed = 1;
				for(int i = 0; i < 6; i++){
					dataToSend[i]= 0xAA;
				}
			}
			if(is_joy_pressed(JOY_RIGHT)){
				joyPressed = 1;
				for(int i = 0; i < 6; i++){
					dataToSend[i]= 0xFF;
				}
			}

			if(joyPressed){
				joyPressed = 0;

				//send First Frame
				// first frame type 0x1, data length: 9d -> 0x009h
				TxData2[0] = 0x10;
				TxData2[1] = 0x09;
				TxData2[2] = 0x2E;	//service ID
				TxData2[3] = 0xF0;
				TxData2[4] = 0x02;
				TxData2[5] = dataToSend[0];
				TxData2[6] = dataToSend[1];
				TxData2[7] = dataToSend[2];

				TxHeader2.DLC = 8;
				HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2);

				//wait Flow Control
				set_timer1(5000); // timeout 5s
				while(dataflag2 == 0 && timer1Flag == 0){}
				if(timer1Flag){
					timer1Flag = 0;
					break;
				}
				//send Consecutive Frame
				// consecutive frame type 0x2, sequence number 0x1
				TxData2[0] = 0x21;
				TxData2[1] = dataToSend[3];
				TxData2[2] = dataToSend[4];
				TxData2[3] = dataToSend[5];

				TxHeader2.DLC = 4;
				HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2);
			}
			// may be wait for pos response
			break;
		case SERVICE_27:
			if(!seedRequested){
				TxData2[0] = 0x02;
				TxData2[1] = 0x27;
				TxData2[2] = 0x01;

				TxHeader2.DLC = 3;
				HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2);
				seedRequested = 1;
			}else if(!keySent){
				if(dataflag2){
					TxData2[0] = 0x06;
					TxData2[1] = 0x22;
					TxData2[2] = 0x02;
					TxData2[3] = RxData2[3] + 1;
					TxData2[4] = RxData2[4] + 1;
					TxData2[5] = RxData2[5] + 1;
					TxData2[6] = RxData2[6] + 1;

					TxHeader2.DLC = 7;
					HAL_CAN_AddTxMessage(&hcan2, &TxHeader2, TxData2, &TxMailbox2);
					keySent = 1;
				}
			}else{
				if(dataflag2){
					// pos response, so we done here
					seedRequested = 0;
					keySent = 0;
				}
			}
			break;
		default:
			break;
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  hcan1.Init.Prescaler = 16;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_1TQ;
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
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 16;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : JOY_CTR_Pin JOY_A_Pin JOY_B_Pin JOY_C_Pin
                           JOY_D_Pin */
  GPIO_InitStruct.Pin = JOY_CTR_Pin|JOY_A_Pin|JOY_B_Pin|JOY_C_Pin
                          |JOY_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB2 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	get_joystick();
	timer_run();
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	ADCValue = HAL_ADC_GetValue(hadc);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if(hcan->Instance == CAN1){
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData)==HAL_OK){
			dataflag = 1;
		}
	}
	if(hcan->Instance == CAN2){
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader2, RxData2)==HAL_OK){
			dataflag2 = 1;
		}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
