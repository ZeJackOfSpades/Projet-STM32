/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum    E_MCP980x_REGISTERS
{
	MCP980x_I2C_REG_TEMP        = (0x00),
	MCP980x_I2C_REG_CONF        = (0x01),
	MCP980x_I2C_REG_TEMPHIST    = (0x02),
	MCP980x_I2C_REG_TEMPLIMIT    = (0x03),

	MCP980x_I2C_RegistersCount

};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCP980x_I2C_ADDR_R    (0x9F)
#define MCP980x_I2C_ADDR_W    (0x9E)

#define DESACTIVE 1
#define ACTIF 0

#define ALERTE_TEMPERATURE 	1
#define FIN_ALERTE_TEMPERATURE	0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint32_t pulseEngine = 300;
volatile uint8_t maxSpeed = 0;
volatile uint8_t desactivate = 0;
volatile uint8_t consigneTemp = 30;
volatile uint8_t consigneHist = 29;
volatile uint8_t flagAlerteTemperature = FIN_ALERTE_TEMPERATURE;
volatile uint8_t bufferReceiveTemp[1]={0};
volatile uint8_t bufferReceiveRegConf[1]={0};
volatile uint8_t bufferReceiveTEMPHIST[1]={0};
volatile uint8_t bufferReceiveTEMPLIMIT[2]={0};
//volatile uint8_t bufferTransmit[2]	= {consigneTemp,0x00};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void BCD_Write(volatile uint8_t _u8Addr, volatile uint8_t _8Data);
static void BCD_Init(void);
static void BCD_Example(void);

void set_ConsigneTemperature(uint16_t temperature);
void set_PWM_TIM_ENGINE(uint32_t Pulse, uint8_t desactivate);
void set_PWM_TIM_BUZZ(uint32_t Pulse);
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


	//uint8_t bufferConf[1]		= {0x28};
	uint8_t bufferConf = 0x68;
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
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	BCD_Init();
	//HAL_GetTick();
	//BCD_Write(0x01,0x0C);
	HAL_Delay(1000);
	BCD_Example();
	HAL_NVIC_SetPriority(EXTI2_IRQn,     5, 0);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_SetPriority(EXTI9_5_IRQn,   5, 0);
	HAL_NVIC_SetPriority(TIM2_IRQn,      6, 0);



	set_ConsigneTemperature(consigneTemp);
	//HAL_Delay(4000);

	//Ce printf sera redirige vers l'UART (redefinition de _write)
	HAL_I2C_Mem_Write(&hi2c1, MCP980x_I2C_ADDR_W, MCP980x_I2C_REG_CONF, 1, &bufferConf, 1, 2000);
	HAL_I2C_Mem_Read(&hi2c1, MCP980x_I2C_ADDR_R, MCP980x_I2C_REG_CONF, 1, bufferReceiveRegConf,1,2000);
	HAL_I2C_Mem_Write(&hi2c1, MCP980x_I2C_ADDR_W, MCP980x_I2C_REG_TEMPLIMIT, 1, (&consigneTemp), 2, 2000);
	HAL_I2C_Mem_Write(&hi2c1, MCP980x_I2C_ADDR_W, MCP980x_I2C_REG_TEMPHIST, 1, (&consigneHist), 2, 2000);
	HAL_I2C_Mem_Read(&hi2c1, MCP980x_I2C_ADDR_R, MCP980x_I2C_REG_TEMPLIMIT, 1, bufferReceiveTEMPLIMIT,2,2000);
	//HAL_I2C_Mem_Read(&hi2c1, MCP980x_I2C_ADDR_R, MCP980x_I2C_REG_TEMPLIMIT, 1, bufferReceiveTEMPLIMIT,2,2000);
	//HAL_Delay(1000);
	BCD_Write(0x01, 0x0A);
	BCD_Write(0x02, 0x0A);
	BCD_Write(0x03, 0x0A);
	BCD_Write(0x04, 0x0A);
	HAL_Delay(1000);
	printf("End of configuration \n");


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		//printf("Temp reg %d, REG conf %d, reg temp HIST %d, temp limit %d\n",bufferReceive[0],bufferReceive[1],bufferReceive[2],bufferReceive[3]);


		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1); //Moteur

		HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2); //BUZZ
		//HAL_Delay(500);
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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
	htim2.Init.Prescaler = 31999;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 2000;
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
	HAL_TIM_Base_Start_IT(&htim2);
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

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 31;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1000;
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
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_PWM_Init(&htim3);

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

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
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : USER_BUTTON_Pin BTN4_Pin BTN3_Pin */
	GPIO_InitStruct.Pin = USER_BUTTON_Pin|BTN4_Pin|BTN3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
	GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : NSS_Pin */
	GPIO_InitStruct.Pin = NSS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : BTN1_Pin BTN2_Pin */
	GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ALERT_TEMP_Pin */
	GPIO_InitStruct.Pin = ALERT_TEMP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(ALERT_TEMP_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 4);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 4);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 4);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, 10);
	return len;
}
static void BCD_Write(volatile uint8_t _u8Addr, volatile uint8_t _u8Data){
	uint8_t l_pu8Word[2];

	l_pu8Word[0] = _u8Addr; //Invert the order if 8 bits mod
	l_pu8Word[1] = _u8Data;

	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, l_pu8Word, 2, HAL_TIMEOUT);
	HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);


}
static void BCD_Init(void){
	BCD_Write(0x0C, 0x01); // Exit the shutdown mode
	BCD_Write(0x0B, 0x03);// 4 digits (from 0 to 3) scan limit
	BCD_Write(0x0A, 0x0B);// Set brightness (11/16)
	BCD_Write(0x09, 0x0F);// Set code B
}

static void BCD_Example(void){
	BCD_Write(0x01, 0x06);
	BCD_Write(0x02, 0x06);
	BCD_Write(0x03, 0x06);
	BCD_Write(0x04, 0x06);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	HAL_I2C_Mem_Read(&hi2c1, MCP980x_I2C_ADDR_R, MCP980x_I2C_REG_TEMP, 1, bufferReceiveTemp, 1, 2000);
	HAL_I2C_Mem_Read(&hi2c1, MCP980x_I2C_ADDR_R, MCP980x_I2C_REG_TEMPLIMIT, 1, bufferReceiveTEMPLIMIT, 1, 2000);
	HAL_I2C_Mem_Read(&hi2c1, MCP980x_I2C_ADDR_R, MCP980x_I2C_REG_TEMPHIST, 1, bufferReceiveTEMPHIST, 1, 2000);
	BCD_Write(0x01, (bufferReceiveTemp[0] > 0 ? 0x0F : 0x0A));
	BCD_Write(0x02, (bufferReceiveTemp[0]/100)%10);
	BCD_Write(0x03, ((bufferReceiveTemp[0]/10)%10));
	BCD_Write(0x04, (bufferReceiveTemp[0]%10));
	printf("Temperature : %d, ALERTE set Temp : %u, HIST %u, engine speed %lu, MAXSPEED : %u\n",bufferReceiveTemp[0],bufferReceiveTEMPLIMIT[0],bufferReceiveTEMPHIST[0], pulseEngine, maxSpeed);

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){ //Routine d'interrupt
	static uint32_t lastTick = 0;
	static uint32_t currentTick = 0;

	currentTick = HAL_GetTick();
	if(currentTick > (lastTick + 300)){
		puts("INTERRUPT\n");
		BCD_Write(0x01, 0x0E);
		BCD_Write(0x02, 0x0E);
		BCD_Write(0x03, 0x0E);
		BCD_Write(0x04, 0x0E);

		printf("GPIO_Pin : %d \n",GPIO_Pin);
		if(GPIO_Pin == BTN1_Pin){
			puts("BTN1\n");
			consigneTemp += 1;
			consigneHist += 1;
			HAL_I2C_Mem_Write(&hi2c1, MCP980x_I2C_ADDR_W, MCP980x_I2C_REG_TEMPLIMIT, 1, (&consigneTemp), 2, 2000);
			HAL_I2C_Mem_Write(&hi2c1, MCP980x_I2C_ADDR_W, MCP980x_I2C_REG_TEMPHIST, 1, (&consigneHist), 2, 2000);
			set_ConsigneTemperature(consigneTemp);

		}
		if(GPIO_Pin == BTN2_Pin){
			puts("BTN2\n");
			if(pulseEngine<1000){
				maxSpeed = 0;
				pulseEngine+=50;
			}
			set_PWM_TIM_ENGINE(pulseEngine, ACTIF);
		}
		if(GPIO_Pin == BTN3_Pin){
			puts("BTN3\n");
			consigneTemp -=1;
			consigneHist -=1;
			HAL_I2C_Mem_Write(&hi2c1, MCP980x_I2C_ADDR_W, MCP980x_I2C_REG_TEMPLIMIT, 1, (&consigneTemp), 2, 2000);
			HAL_I2C_Mem_Write(&hi2c1, MCP980x_I2C_ADDR_W, MCP980x_I2C_REG_TEMPHIST, 1, (&consigneHist), 2, 2000);
			set_ConsigneTemperature(consigneTemp);
			//set_PWM_TIM_BUZZ(0);

		}
		if(GPIO_Pin == BTN4_Pin){
			puts("BTN4\n");
			if(pulseEngine>450)
				pulseEngine-=50;
			set_PWM_TIM_ENGINE(pulseEngine, ACTIF);
		}
		if(GPIO_Pin == USER_BUTTON_Pin){
			if(0 == maxSpeed){
				set_PWM_TIM_ENGINE(1000, ACTIF);
				maxSpeed = 1 ;
			}else{
				set_PWM_TIM_ENGINE(pulseEngine, ACTIF);
				maxSpeed = 0 ;
			}
		}
		if(GPIO_Pin == ALERT_TEMP_Pin){
			if(flagAlerteTemperature == FIN_ALERTE_TEMPERATURE){
				puts("ALERTE TEMPERATURE ! \n");
				flagAlerteTemperature = ALERTE_TEMPERATURE;
				set_PWM_TIM_BUZZ(600);
				set_PWM_TIM_ENGINE(800,ACTIF);
			}else{
				flagAlerteTemperature = FIN_ALERTE_TEMPERATURE;
				puts("FIN ALERTE TEMP\n");
				set_PWM_TIM_BUZZ(0);
				set_PWM_TIM_ENGINE(0,DESACTIVE);
			}
		}

		lastTick = HAL_GetTick();
	}
	//HAL_Delay(5000);
}

void set_PWM_TIM_ENGINE(uint32_t Pulse, uint8_t desactivate){
	TIM_OC_InitTypeDef sConfigOC = {0};

	uint32_t l_u32Pulse = 0;

	if(Pulse < 300 && (desactivate != DESACTIVE)){
		puts("Pulse minimum for the engine is 300\n");
		l_u32Pulse = 300;

	}else if(Pulse > 1000 && (desactivate != DESACTIVE)){
		maxSpeed = 1;
		puts("MAX speed reached ! \n");
	}else{
		l_u32Pulse = Pulse;
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = l_u32Pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

}
void set_PWM_TIM_BUZZ(uint32_t Pulse){
	TIM_OC_InitTypeDef sConfigOC = {0};
	//uint32_t l_u32Pulse;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = Pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
}
// D7 | D6 | D5 | D4 | D3 | D2 | D1 | D0
// DP | A  | B  | C  | D  | E  | F  | G
// 1    1    0    0    1    1    1    0
void set_ConsigneTemperature( uint16_t temperature){
	//BCD_Write(0x09, 0x00); //set not CODE B
	//BCD_Write(0x01, 0b00000000); //Simply for testing
	//BCD_Write(0x01, 0b11100111); //Simply for testing = P.
	//BCD_Write(0x01, 0b11001110); // affiche un C sur le premier digit avec le DP
	//BCD_Write(0x09, 0x0F); //set Code B
	BCD_Write(0x01, 0xFE); //P.
	BCD_Write(0x02, ((temperature/100)%10));
	BCD_Write(0x03, ((temperature/10)%10));
	BCD_Write(0x04, (temperature%10));
	//HAL_I2C_Mem_Read(&hi2c1, MCP980x_I2C_ADDR_R, MCP980x_I2C_REG_TEMPLIMIT, 1, bufferReceiveTEMPLIMIT,2,2000);
	//HAL_I2C_Mem_Write(&hi2c1, MCP980x_I2C_ADDR_W, MCP980x_I2C_REG_TEMPLIMIT, 1, (&consigneTemp), 2, 2000);

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	//TODO Fix before release
	while(1);
	/* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
