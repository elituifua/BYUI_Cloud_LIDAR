#pragma once

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






void wait_cycles(uint32_t cycles){
	while (cycles-- > 0){
		__asm__ volatile ("nop");
	} 				// @ 8MHz, each clock cycle is 125 ns
}


void SPI_Write_8(uint8_t data)
{
    for (int i = 0; i < 8; i++)
    {
        // Set MOSI according to the most significant bit of data
        HAL_GPIO_WritePin(GPIOA, Din_Pin, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1; // Shift data left

        // Toggle SCK
        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_SET);
        wait_cycles(4); // Small delay to simulate clock
        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_RESET);
        wait_cycles(4);
    }
}

void SPI_Write_24(uint32_t data)
{
    for (int i = 0; i < 24; i++)
    {
        // Set MOSI according to the most significant bit of data
        HAL_GPIO_WritePin(GPIOA, Din_Pin, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        data <<= 1; // Shift data left

        // Toggle SCK
        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_SET);
        wait_cycles(4); // Small delay to simulate clock
        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_RESET);
        wait_cycles(4);
    }
}

uint8_t SPI_Read_8(void)
{
    uint8_t data = 0;
    for (int i = 0; i < 8; i++)
    {
        data <<= 1; // Shift data left

        // Toggle SCK
        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_SET);
        wait_cycles(4);

        // Read MISO
        if (HAL_GPIO_ReadPin(GPIOB, Dout_Pin) == GPIO_PIN_SET)
        {
            data |= 0x01;
        }

        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_RESET);
        wait_cycles(4);
    }
    return data;
}

uint32_t SPI_Read_24(void)
{
    uint32_t data = 0;
    for (int i = 0; i < 24; i++)
    {
        data <<= 1; // Shift data left

        // Toggle SCK
        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_SET);
        wait_cycles(4);

        // Read MISO
        if (HAL_GPIO_ReadPin(GPIOB, Dout_Pin) == GPIO_PIN_SET)
        {
            data |= 0x01;
        }

        HAL_GPIO_WritePin(GPIOA, SCLK_Pin, GPIO_PIN_RESET);
        wait_cycles(4);
    }
    return data;
}

uint32_t TDC7200_Read_Register(uint8_t reg)
{
    uint8_t txData = (reg & 0x3F);
    uint32_t rxData = 0;

    HAL_GPIO_WritePin(GPIOA, CS_N_Pin, GPIO_PIN_RESET); // CS low

    if (reg <= 0x09){

        SPI_Write_8(txData); // Send register address
        rxData = SPI_Read_8(); // Read data

    }
    else{

        SPI_Write_8(txData); // Send register address
        rxData = SPI_Read_24(); // Read data

    }

    HAL_GPIO_WritePin(GPIOA, CS_N_Pin, GPIO_PIN_SET); // CS high

    return rxData;
}

void TDC7200_Write_Register(uint8_t reg, uint32_t value)
{
    uint8_t txData = (reg & 0x3F) | 0x40; // Ensure bit 6 is set for write

    HAL_GPIO_WritePin(GPIOA, CS_N_Pin, GPIO_PIN_RESET); // CS low

    if (reg <= 0x09){

        SPI_Write_8(txData); // Send register address
        SPI_Write_8((uint8_t) value); // Send value

    }

    else{

    SPI_Write_8(txData); // Send register address
    SPI_Write_24(value); // Send value

    }

    HAL_GPIO_WritePin(GPIOA, CS_N_Pin, GPIO_PIN_SET); // CS high
}
