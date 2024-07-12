/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "spi.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UART_DELAY 100 // wait max of 100 ms between frames in message
#define MAX_MESSAGE_SIZE 100 // 100 characters maximum message size
#define MAX_NEW_POT_VALUE_SIZE 3 // 3 characters maximum potentiometer value size
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
int  new_pot_value;
char message[MAX_MESSAGE_SIZE];
char distance[MAX_MESSAGE_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Set_Pot_Value(uint8_t pot_value)
{
	/* This function will take in a 8 bit value that will determine the value of the potentiometer.
	 * The DigiPot needs three things to update 1. Slave Address, 2. Register Address, and 3. Pot Value
	 * The Slave address (8bit) is comprised of the address (7bits) and a read/write bit (1bit lsb)
	 * The register address for the pot is 0b 0000 0000
	 * Potentiometer Values can range from 0x00 to 0xFF
	 */

	uint8_t tx_data[2];  // Buffer to hold the data to be transmitted
	uint8_t SLAVE_ADDRESS = (0x2E << 1) ; //This will need to be changed.
	// Populate the data buffer
	tx_data[0] = 0b00000000;  // Register address to write to
	tx_data[1] = pot_value;   // 8-bit word to write //SEE Resistance Value table in Data sheet.

	// Perform I2C transmission
	HAL_I2C_Master_Transmit(&hi2c1, (uint8_t)SLAVE_ADDRESS, tx_data, 2, HAL_MAX_DELAY);
}

void Intialize_TDC(void)
{
	  char thing[30] = "Yeetus";
	  HAL_UART_Transmit(&huart2, (unsigned char*) thing, strlen(thing), UART_DELAY);

	HAL_GPIO_WritePin(GPIOA, CS_N_Pin, GPIO_PIN_SET);
	// Set Enable Pin
 	HAL_GPIO_WritePin(GPIOA, Enable_Pin, GPIO_PIN_SET);

	// wait at least 1.5 ms (12,000 clock cycles) for LDO_SET2 (see datasheet 8.4.7)
	wait_cycles(24000);

	// Set tdc to mode 2
	// set force calibration to 1
	uint32_t config1 = TDC7200_Read_Register(TDC_CONFIG1) | 0x12;
	TDC7200_Write_Register(TDC_CONFIG1, config1);
	config1 = TDC7200_Read_Register(TDC_CONFIG1);

	// set calibration2_periods to b'11
	uint32_t config2 = TDC7200_Read_Register(TDC_CONFIG2) | 0xC0;
	TDC7200_Write_Register(TDC_CONFIG2, config2);


}

double take_measurement(){
	// Set START_MEAS bit to 1


	uint32_t config_value = TDC7200_Read_Register(TDC_CONFIG1);
	config_value |= 0x01;
	TDC7200_Write_Register(TDC_CONFIG1, config_value);

	wait_cycles(400);
	//config_value = TDC7200_Read_Register(TDC_CONFIG1); // for testing

	// Wait for trig
//	while (HAL_GPIO_ReadPin(GPIOA, Trigg_Pin) != GPIO_PIN_SET)
//		{
//			wait_cycles(1);
//		}


    //when trig goes high, set start_pin high and laser control pin high
	HAL_GPIO_WritePin(GPIOC, Laser_Control_Pin, GPIO_PIN_SET); // Laser High
	//wait_cycles(1);//Optimize delay for accurate timing //less than then 33ns would be great.
	HAL_GPIO_WritePin(GPIOA, Start_Pin, GPIO_PIN_SET); // Start High

	//wait_cycles(100);
	//HAL_GPIO_WritePin(GPIOC, Test_Output_Pin, GPIO_PIN_SET); // Stop High

    // wait for interrupt
//    while (HAL_GPIO_ReadPin(GPIOA, Interrupt_Pin) == GPIO_PIN_SET)
//        {
//            wait_cycles(1);
//        }
wait_cycles(64000);
	// read result
    HAL_GPIO_WritePin(GPIOA, Start_Pin, GPIO_PIN_RESET); // Start low
    HAL_GPIO_WritePin(GPIOC, Laser_Control_Pin, GPIO_PIN_RESET); // Laser low
	//HAL_GPIO_WritePin(GPIOC, Test_Output_Pin, GPIO_PIN_RESET); // for testing

    // Calculate Time of Flight
    wait_cycles(5000);
    double time1 = TDC7200_Read_Register(TDC_TIME1);
    double time2 = TDC7200_Read_Register(TDC_TIME2);
    double cal1 = TDC7200_Read_Register(TDC_CALIBRATION1);
    double cal2 = TDC7200_Read_Register(TDC_CALIBRATION2);
    double clock_count1 = TDC7200_Read_Register(TDC_CLOCK_COUNT1);
    const double cal2_periods = 40;
    const double clk_period = 0.000000125;

    double cal_count = (cal2-cal1)/(cal2_periods - 1);

    double norm_lsb = clk_period / cal_count;
    double tof = ((time1 - time2) * norm_lsb) + (clock_count1 * clk_period);
    return tof;

}

double calculate_offset(double measured_time){
	// Calculate offset for time for laser to turn on and any other delays in the circuit
	return measured_time;
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  Intialize_TDC();

  wait_cycles(1000000);

  Set_Pot_Value(108);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // check for message on USART2
//	  if (USART2->ISR & UART_FLAG_RXNE){
//		  // zero out message array
//	  	  memset(distance, 0, sizeof(message));
	  	  // get message from USART2
//	  	  HAL_UART_Receive(&huart2, (unsigned char*) message, sizeof(message)-1, UART_DELAY);
	  	  // send the same message to USART1
//	  	  HAL_UART_Transmit(&huart1, (unsigned char*) message, strlen(message), UART_DELAY);
	  	  // check for message on USART1
//	  if (USART1->ISR & UART_FLAG_RXNE){
//		  // zero out message array
//	 	  memset(message, 0, sizeof(message));
	  	  // get message from USART1
//	  	  HAL_UART_Receive(&huart1, (unsigned char*) message, sizeof(message)-1, UART_DELAY);
	  	  // send the same message to USART2


//	  	  if (strcmp(message,"Set_pot_val") == 0){
//	  		  memset(message, 0, sizeof(message));
//	  		  strncpy(message, "Please enter pot value (1-128): ", MAX_MESSAGE_SIZE);
//	  		  if (1 <= atoi(message) <= 128){
//	  			  new_pot_value = atoi(message);
//	  		  }
//	  		  else{
//	  			  strncpy(message, "Please use the set_pot_val command again with valid pot value.", MAX_MESSAGE_SIZE);
//	  		  }
//	  	  }
//	  	  else if (strcmp(message, "Start_measure") == 0){

	  wait_cycles(1000000);
	  double tof = take_measurement();
	  tof = calculate_offset(tof);
	  double distance_meas = tof*299792458*0.5;
	  char distance_meas_str[MAX_MESSAGE_SIZE];
	  snprintf(distance_meas_str, MAX_MESSAGE_SIZE, "%f%s", distance_meas,"\r\n");
	  //char nl_character[MAX_MESSAGE_SIZE] ;//
	  //snprintf(nl_character, MAX_MESSAGE_SIZE, "%s", "\n\0");
//	  strncpy(distance, distance_meas_str, MAX_MESSAGE_SIZE);
	  HAL_UART_Transmit(&huart2, (unsigned char*) distance_meas_str, strlen(distance_meas_str), UART_DELAY);


	  //HAL_UART_Transmit(&huart2, (unsigned char*) nl_character, strlen(nl_character), UART_DELAY);

  }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


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
  RCC_OscInitStruct.PLL.PLLN = 8;
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
  hi2c1.Init.Timing = 0xA0006AFF;
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Laser_Control_Pin|Test_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SCLK_Pin|CS_N_Pin|Din_Pin|Start_Pin
                          |Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SDA_dp_Pin|SCL_dp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Laser_Control_Pin Test_Output_Pin */
  GPIO_InitStruct.Pin = Laser_Control_Pin|Test_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Interrupt_Pin */
  GPIO_InitStruct.Pin = Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Interrupt_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SCLK_Pin CS_N_Pin Din_Pin Start_Pin
                           Enable_Pin */
  GPIO_InitStruct.Pin = SCLK_Pin|CS_N_Pin|Din_Pin|Start_Pin
                          |Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SDA_dp_Pin SCL_dp_Pin */
  GPIO_InitStruct.Pin = SDA_dp_Pin|SCL_dp_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Trigg_Pin */
  GPIO_InitStruct.Pin = Trigg_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Trigg_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Dout_Pin */
  GPIO_InitStruct.Pin = Dout_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Dout_GPIO_Port, &GPIO_InitStruct);

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
