/* Includes -------------------------------------------------------*/
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

/* Global variables -----------------------------------------------*/
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* Private function prototypes ------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
void newStep(void);

/* User code ------------------------------------------------------*/
#define MPU6050_ADDR       0xD0
#define PWR_MGMT_1_REG     0x6B
#define SMPLRT_DIV_REG     0x19
#define ACCEL_CONFIG_REG   0x1C
#define ACCEL_XOUT_H_REG   0x3B
#define LPF_ENABLE         0x1A


int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
int16_t X_RAW, Y_RAW, Z_RAW;
uint8_t Rec_Data[6];
volatile float x,y,z = 0;
uint8_t i;
volatile float xaccl[5000] = {0};
volatile float yaccl[5000] = {0};
volatile float zaccl[5000] = {0};
volatile float x_th, y_th, z_th;
volatile int steps = 0;
long old_time = 0, new_time, current_time;

void MPU_Init (void)
{
	uint8_t Data;

	// Configure the MPU6050 Power Management 1 Register
    Data = 0x88;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
	HAL_Delay(100);

    // Configure the MPU6050 Power Management 1 Register
	Data = 0x08;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
    HAL_Delay(100);

	// Set DATA RATE of 50Hz by writing SMPLRT_DIV register
	Data = 0x19;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

	// Configure the MPU6050 Accelerometer Configuration Register
	Data = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

	// Configure the MPU6050 LPF Register
	Data = 0x06;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, LPF_ENABLE, 1, &Data, 1, 1000);
}

void CountSteps(void)
{
	volatile float x_max = 0.0, x_min = 0.0, y_max = 0.0, y_min = 0.0, z_max = 0.0, z_min = 0.0;

	  for (int a = 0; a < 5000 ;a++)
	  {
			  HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

			  Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
			  Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
			  Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
			  xaccl[a] = Accel_X_RAW/16384.0;
			  yaccl[a] = Accel_Y_RAW/16384.0;
			  zaccl[a] = Accel_Z_RAW/16384.0;

	          if (xaccl[a] > x_max)
	        	  x_max = xaccl[a];
	          if (xaccl[a] < x_min)
	        	  x_min = xaccl[a];

	          if (yaccl[a] > y_max)
	        	  y_max = yaccl[a];
	          if (yaccl[a] < y_min)
	        	  y_min = yaccl[a];

	          if (zaccl[a] > z_max)
	        	  z_max = zaccl[a];
	          if (zaccl[a] < z_min)
	        	  z_min = zaccl[a];
	  }

      x_th = (x_max + x_min) / 2;
      y_th = (y_max + y_min) / 2;
      z_th = (z_max + z_min) / 2;

	while(i < 50)
	{
        HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
		X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
		Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
		Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
		x = X_RAW/16384.0;
		y = Y_RAW/16384.0;
		z = Z_RAW/16384.0;

        if (x > y)		 			// x > y
         {
           if (x > z)				// x is the largest
            {
              if (x > x_th)
              newStep();
            }
           else
            {
              if (z > z_th)			// z is the largest
            newStep();
            }
         }
        else
         {
           if (y > z)				// y > x
            {
              if (y > y_th)			// y is the largest
              newStep();
            }
           else
            {
              if(z > z_th)			// z is the largest
               newStep();
            }
         }


		i++;
	}
}

void newStep(void)
{
        steps++;
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* Initialize the MPU6050 Module */
  MPU_Init();

  /* Infinite loop */

  while(1)
  {
	  CountSteps();
	  i = 0;
  }
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC13_Pin */
  GPIO_InitStruct.Pin =  GPIO_PIN_13;
  GPIO_InitStruct.Mode = MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
