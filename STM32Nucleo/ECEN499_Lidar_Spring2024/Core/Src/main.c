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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TDC_CONFIG1                       0x00
#define TDC_CONFIG2                       0x01
#define TDC_INT_STATUS                    0x02
#define TDC_INT_MASK                      0x03
#define TDC_COARSE_CNTR_OVF_H             0x04
#define TDC_COARSE_CNTR_OVF_L             0x05
#define TDC_CLOCK_CNTR_OVF_H              0x06
#define TDC_CLOCK_CNTR_OVF_L              0x07
#define TDC_CLOCK_CNTR_STOP_MASK_H        0x08
#define TDC_CLOCK_CNTR_STOP_MASK_L        0x09
#define TDC_TIME1                         0x10
#define TDC_CLOCK_COUNT1                  0x11
#define TDC_TIME2                         0x12
#define TDC_CLOCK_COUNT2                  0x13
#define TDC_TIME3                         0x14
#define TDC_CLOCK_COUNT3                  0x15
#define TDC_TIME4                         0x16
#define TDC_CLOCK_COUNT4                  0x17
#define TDC_TIME5                         0x18
#define TDC_CLOCK_COUNT5                  0x19
#define TDC_TIME6                         0x1A
#define TDC_CALIBRATION1                  0x1B
#define TDC_CALIBRATION2                  0x1C

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void wait_cycles(uint32_t cycles){
	while (cycles-- > 0){
		__asm__ volatile ("nop");
	}
}


void SPI_Write(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        // Set MOSI according to the most significant bit of data
        HAL_GPIO_WritePin(GPIOA, Din_Pin, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1; // Shift data left

        // Toggle SCK
        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_SET);
        wait_cycles(10); // Small delay to simulate clock
        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_RESET);
        wait_cycles(10);
    }
}

uint32_t SPI_Read(void)
{
    uint32_t data = 0;
    for (int i = 0; i < 24; i++)
    {
        data <<= 1; // Shift data left

        // Toggle SCK
        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_SET);
        wait_cycles(10);

        // Read MISO
        if (HAL_GPIO_ReadPin(GPIOB, Dout_Pin) == GPIO_PIN_SET)
        {
            data |= 0x01;
        }

        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_RESET);
        wait_cycles(10);
    }
    return data;
}

uint32_t TDC7200_Read_Register(uint8_t reg)
{
    uint8_t txData = (reg & 0x3F);
    uint32_t rxData = 0;

    HAL_GPIO_WritePin(GPIOA, CS_N_Pin, GPIO_PIN_RESET); // CS low

    SPI_Write(txData); // Send register address
    rxData = SPI_Read(); // Read data

    HAL_GPIO_WritePin(GPIOA, CS_N_Pin, GPIO_PIN_SET); // CS high

    return rxData;
}

void TDC7200_Write_Register(uint8_t reg, uint8_t value)
{
    uint8_t txData = (reg & 0x1F) + 0x40; // Ensure bit 6 is set for write

    HAL_GPIO_WritePin(GPIOA, CS_N_Pin, GPIO_PIN_RESET); // CS low

    SPI_Write(txData); // Send register address
    SPI_Write(value); // Send value

    HAL_GPIO_WritePin(GPIOA, CS_N_Pin, GPIO_PIN_SET); // CS high
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
  /* USER CODE BEGIN 2 */

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
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CLK_TriState_Pin|CS_N_Pin|SCLK_Pin|Din_Pin
                          |Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SDA_dp_Pin|SCL_dp_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Start_GPIO_Port, Start_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CLK_TriState_Pin CS_N_Pin SCLK_Pin Din_Pin
                           Enable_Pin */
  GPIO_InitStruct.Pin = CLK_TriState_Pin|CS_N_Pin|SCLK_Pin|Din_Pin
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

  /*Configure GPIO pin : Start_Pin */
  GPIO_InitStruct.Pin = Start_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Start_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Trig_LOOK_AT_Pin */
  GPIO_InitStruct.Pin = Trig_LOOK_AT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Trig_LOOK_AT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Dout_Pin Interrupt_Pin */
  GPIO_InitStruct.Pin = Dout_Pin|Interrupt_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
