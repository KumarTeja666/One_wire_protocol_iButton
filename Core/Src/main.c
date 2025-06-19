/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
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
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t ROM_ID[8]={0};
uint8_t eeprom_data[512];  // Array to store 512 bytes of data
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim6,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim6) < us);  // wait for the counter to reach the us input in the parameter
}

void OneWire_Pin_Output(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = iB_Data_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(iB_Data_GPIO_Port, &GPIO_InitStruct);
}

void OneWire_Pin_Input(void) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = iB_Data_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(iB_Data_GPIO_Port, &GPIO_InitStruct);
}

void OneWire_WriteBit(uint8_t bit) {
	OneWire_Pin_Output();
	HAL_GPIO_WritePin(iB_Data_GPIO_Port, iB_Data_Pin, GPIO_PIN_RESET);

	if (bit) {
		delay_us(6);
		OneWire_Pin_Input();
		delay_us(64);
	} else {
		delay_us(60);
		OneWire_Pin_Input();
		delay_us(10);
	}
}

uint8_t OneWire_ReadBit(void) {
	uint8_t bit;

	OneWire_Pin_Output();
	HAL_GPIO_WritePin(iB_Data_GPIO_Port, iB_Data_Pin, GPIO_PIN_RESET);
	delay_us(6);
	OneWire_Pin_Input();
	delay_us(9);

	bit = HAL_GPIO_ReadPin(iB_Data_GPIO_Port, iB_Data_Pin);
	delay_us(55);

	return bit;
}

void OneWire_WriteByte(uint8_t byte) {
	for (int i = 0; i < 8; i++) {
		OneWire_WriteBit(byte & 0x01);
		byte >>= 1;
	}
}

uint8_t OneWire_ReadByte(void) {
	uint8_t byte = 0;

	for (int i = 0; i < 8; i++) {
		byte |= (OneWire_ReadBit() << i);
	}

	return byte;
}

uint8_t OneWire_Reset(void) {
	uint8_t presence;

	OneWire_Pin_Output();
	HAL_GPIO_WritePin(iB_Data_GPIO_Port, iB_Data_Pin, GPIO_PIN_RESET);
	delay_us(480); // Pull low for 480us


	OneWire_Pin_Input();  // Release the line
	delay_us(70);         // Wait 70us

	presence = HAL_GPIO_ReadPin(iB_Data_GPIO_Port, iB_Data_Pin);
	delay_us(410);        // Wait for rest of the timeslot
	if(presence==0)
	{
		HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, 1);
	}
	return (presence == GPIO_PIN_RESET) ? 1 : 0; // 1 if device is present
}

void OneWire_MatchROM(uint8_t* romID) {
	OneWire_WriteByte(0x55);  // Match ROM command

	// Write each byte of the ROM ID
	for (uint8_t i = 0; i < 8; i++) {
		OneWire_WriteByte(romID[i]);
	}
}

void OneWire_WriteBytes(uint8_t* data, uint16_t length) {
	for (uint16_t i = 0; i < length; i++) {
		OneWire_WriteByte(data[i]);
	}
}

void OneWire_sendReadMemoryCMD(uint16_t address) {
	uint8_t cmd[3];

	// Send the Read Memory command (0xF0)
	cmd[0] = 0xF0;

	// Address (2 bytes: low and high byte)
	cmd[1] = (uint8_t)(address & 0xFF);       // Low byte of address
	cmd[2] = (uint8_t)((address >> 8) & 0xFF); // High byte of address

	// Write the Read Memory command to the iButton
	OneWire_WriteBytes(cmd, sizeof(cmd));
}

void OneWire_ReadBytes(uint8_t* buffer, uint16_t length)
{
	for (uint16_t i = 0; i < length; i++)
	{
		buffer[i] = OneWire_ReadByte();
	}
}

// Function to read all 512 bytes (4K) of memory from the DS1973 EEPROM
void OneWire_ReadAllMemory(uint8_t *eeprom_data) {
	for (uint8_t page = 0; page < 32; page++) {
		uint16_t addr = page * 32;

		if (OneWire_Reset()){

			OneWire_WriteByte(0xCC); // Skip ROM
			OneWire_WriteByte(0xF0); // Read Memory
			OneWire_WriteByte(addr & 0xFF);        // Address LSB
			OneWire_WriteByte((addr >> 8) & 0xFF); // Address MSB

			for (uint8_t i = 0; i < 32; i++) {
				eeprom_data[addr + i] = OneWire_ReadByte();
			}
		}
		HAL_Delay(10); // Delay 10ms after each page read
	}
}

uint8_t DS1973_WritePage(uint8_t page_num, uint8_t *data) {
	uint16_t address = page_num * 32;  // since each page = 32 bytes
	uint8_t TA1 = address & 0xFF;
	uint8_t TA2 = (address >> 8) & 0xFF;

    uint8_t ES;

    // STEP 1: Reset and Skip ROM
    if (!OneWire_Reset()) return 0;
    OneWire_WriteByte(0xCC);  // SKIP ROM

    // STEP 2: Write Scratchpad
    OneWire_WriteByte(0x0F);  // WRITE SCRATCHPAD
    OneWire_WriteByte(TA1);   // TA1
    OneWire_WriteByte(TA2);   // TA2

    for (int i = 0; i < 32; i++) {
        OneWire_WriteByte(data[i]);
    }

    // STEP 3: Reset and Read Scratchpad
    if (!OneWire_Reset()) return 0;
    OneWire_WriteByte(0xCC);  // SKIP ROM
    OneWire_WriteByte(0xAA);  // READ SCRATCHPAD

    uint8_t rTA1 = OneWire_ReadByte(); // TA1
    uint8_t rTA2 = OneWire_ReadByte(); // TA2
    ES = OneWire_ReadByte();           // Ending offset (ES)

    // (Optional: Read the 16 data bytes back for verify)
    for (int i = 0; i < 16; i++) OneWire_ReadByte();

    // STEP 4: Reset and Copy Scratchpad
    if (!OneWire_Reset()) return 0;
    OneWire_WriteByte(0xCC);  // SKIP ROM
    OneWire_WriteByte(0x55);  // COPY SCRATCHPAD
    OneWire_WriteByte(rTA1);  // TA1
    OneWire_WriteByte(rTA2);  // TA2
    OneWire_WriteByte(ES);    // Ending offset byte

    // STEP 5: Wait for EEPROM write (tPROG max = 5-10 ms)
    HAL_Delay(10);

    return 1;  // Success
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
	MX_USART2_UART_Init();
	MX_TIM6_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim6);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(iB_Data_GPIO_Port, iB_Data_Pin, 1);
	HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, 0);
	HAL_Delay(5000);

	if(OneWire_Reset())
	{
		OneWire_WriteByte(0xCC);
	}
	else
	{
		HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, 0);
	}


	uint8_t my_data[32] = {
	  'H','e','l','l','o','_','S','T','M','3','2','_','E','E','P','R','O','M'
	};

	DS1973_WritePage(15, my_data);  // Write to page 0

//	for (int i = 0; i < 8; i++) {
//		ROM_ID[i] = OneWire_ReadByte();
//	}



	// Read all EEPROM data (512 bytes)
	OneWire_ReadAllMemory(eeprom_data);


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
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 71;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 999;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|iB_Data_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(G_LED_GPIO_Port, G_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : G_LED_Pin */
	GPIO_InitStruct.Pin = G_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(G_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : iB_Data_Pin */
	GPIO_InitStruct.Pin = iB_Data_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(iB_Data_GPIO_Port, &GPIO_InitStruct);

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
