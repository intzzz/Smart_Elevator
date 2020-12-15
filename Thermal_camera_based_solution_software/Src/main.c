/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define SLAVE_ADDR 0x0A // 7-bit, right aligned
#define MATRIX_SIZE 32*32
#define N_READ ((MATRIX_SIZE + 1) * 2 + 1)

#define NB_OK "OK"
#define NB_ERROR "ERROR"

#define AT "AT"

uint8_t d6t_ReadBuf[N_READ];

int human_temperature = 250; // Min human temperature regardless of distance and position
int min_pixels_for_detection = 20; // Minimum cluster of pixels for human detection
int max_frames_to_determine_passerby = 4; // Max frames to distinguish passenger and passer-by

int test_error = 0; //debugging
int pTAT; // Inside temperature of the sensor
int temp[MATRIX_SIZE]; // Temperature into one array
int temp_matrix[32][32]; // Temperature into matrix array
uint8_t PEC; // Error check
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float find_avg_temp(uint8_t buf[]){ // Get average temperature of the room
	float avg = 0;
	for(int i = 0; i < MATRIX_SIZE; i++){
		avg += buf[i];
	}
	avg = avg / MATRIX_SIZE;
	return avg;
}

void getTemperature(){ // Get temperature into one array through I2C
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c1, (SLAVE_ADDR << 1), 0x4D, 1, d6t_ReadBuf, sizeof(d6t_ReadBuf), 500);
	if (status != HAL_OK){
		return;
	}
	pTAT = 256*d6t_ReadBuf[1] + d6t_ReadBuf[0];
	int j = 2;
	for (int i = 0; i < MATRIX_SIZE; i++){
		temp[i] = (256*d6t_ReadBuf[j + 1] + d6t_ReadBuf[j]);
		j = j + 2;
	}
	PEC = d6t_ReadBuf[N_READ - 1];
}

void getTemperature_matrix(){ // Get temperature into matrix array through I2C
	HAL_StatusTypeDef status = HAL_OK;
	status = HAL_I2C_Mem_Read(&hi2c1, (SLAVE_ADDR << 1), 0x4D, 1, d6t_ReadBuf, sizeof(d6t_ReadBuf), 500);
	if (status != HAL_OK){
		return;
	}
	pTAT = 256*d6t_ReadBuf[1] + d6t_ReadBuf[0];
	int j = 2;
	int k = 0;
	int l = 0;
	for (int i = 0; i < MATRIX_SIZE; i++){
		temp_matrix[k][l] = (256*d6t_ReadBuf[j + 1] + d6t_ReadBuf[j]);
		j = j + 2;
		l++;
		if (l == 32){
			l = 0;
			k++;
		}
	}
	PEC = d6t_ReadBuf[N_READ - 1];
}

//Area of Interest consists of 3 rectangles
int detect_human_in_AOI(){
	int detected_pixels = 0;
// Rectangle 1
	for(int i = 4; i <= 21; i++){
		for(int j = 10; j <= 21; j++){
			if(temp_matrix[i][j] >= human_temperature){ // If pixels have higher than human temperature
				detected_pixels++;
			}
			if (detected_pixels >= min_pixels_for_detection){ // If cluster of pixels detected with high enough temperature
				return 1;
			}
		}
	}
// Rectangle 2
	for(int i = 22; i <= 26; i++){
		for(int j = 9; j <= 22; j++){
			if(temp_matrix[i][j] >= human_temperature){ // If pixels have higher than human temperature
				detected_pixels++;
			}
			if (detected_pixels >= min_pixels_for_detection){ // If cluster of pixels detected with high enough temperature
				return 1;
			}
		}
	}
// Rectangle 3
	for(int i = 27; i <= 31; i++){
		for(int j = 8; j <= 23; j++){
			if(temp_matrix[i][j] >= human_temperature){ // If pixels have higher than human temperature
				detected_pixels++;
			}
			if (detected_pixels >= min_pixels_for_detection){ // If cluster of pixels detected with high enough temperature
				return 1;
			}
		}
	}
	return 0;
}

uint8_t calc_crc(uint8_t data) { // Calculate new crc
    int index;
    uint8_t tmp;
    for (index = 0; index < 8; index++) {
        tmp = data;
        data <<= 1;
        if (tmp & 0x80) {data ^= 0x07;}
    }
    return data;
}

int D6T_checkPEC(uint8_t buf[], int n) { // Check if new crc and read crc match
    int i;
    uint8_t crc = calc_crc((SLAVE_ADDR << 1) | 1);  // I2C Read address (8bit)
    for (i = 0; i < n; i++) {
        crc = calc_crc(buf[i] ^ crc);
    }
    if (crc != PEC) {
    	return 0;
    }
    else
    	return 1;
}

void send_temperature_USB_UART(){ // make new data format to send to Raspberry for visualizing and analyzing
	int k = 0;
	char buffer[5121] = "\0";
	char buff3[4] = "\0";
	getTemperature(); // Get temperature from sensor
	if (D6T_checkPEC(d6t_ReadBuf, N_READ - 1) == 1){ // Check if no errors
	  for(int i = 0; i < MATRIX_SIZE; i++){
		  sprintf(buff3, "%d", temp[i]);
		  if(temp[i] >= 100){
			  buffer[k++] = buff3[0];
			  buffer[k++] = buff3[1];
			  buffer[k++] = '.';
			  buffer[k++] = buff3[2];
			  buffer[k++] = ',';
		  }
		  else if(temp[i] >= 10 && temp[i] < 100){
			  buffer[k++] = '0';
			  buffer[k++] = buff3[0];
			  buffer[k++] = '.';
			  buffer[k++] = buff3[1];
			  buffer[k++] = ',';
		  }
		  else{
			  buffer[k++] = '0';
			  buffer[k++] = '0';
			  buffer[k++] = '.';
			  buffer[k++] = buff3[0];
			  buffer[k++] = ',';
		  }
	  }
	  buffer[5119] = '\n'; // Ends with newline

	  if(HAL_UART_Transmit(&huart2, (uint8_t*)buffer, 5120, 500)!= HAL_OK) // Transmit data to Raspberry through UART
	  {
		Error_Handler();
	  }
	}
	HAL_Delay(300); // Updates data every 300 ms
}



void thermal_camera(){ // Detect elevator passengers, proposed solution
	getTemperature_matrix();
	if (detect_human_in_AOI() == 1){
	  i++;
	}
	else{
	  i = 0;
	}
	if(i > max_frames_to_determine_passerby){ // If human detected in more than 4 consecutive frames, it is en elevator passenger
	  i = 0;
	  //Send data
	  // communicate with MCU etc
	}
	HAL_delay(300); // Temperature data update rate
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  int i = 0;
  /* USER CODE END 2 */
 
 

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x00100413;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** I2C Fast mode Plus enable 
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C1);
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

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
	test_error = 1;
	//while(1){
	 // HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
	 // HAL_Delay(1000);
	//}
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
