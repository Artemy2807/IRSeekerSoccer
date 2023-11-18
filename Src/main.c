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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define M_PI			(3.1415926535897931159979635f)
#define TODEG			(180.0f / M_PI)
#define TORAD			(M_PI / 180.0f)

#define HIGH_TSSP	92
#define LOW_TSSP	18
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */
const uint8_t tssp_size = 32;
GPIO_TypeDef *tssp_pin_base[tssp_size] =
{
	GPIOB,
	GPIOB,
	GPIOC,
	GPIOC,
	GPIOA,
	GPIOA,
	GPIOA,
	GPIOA,
	GPIOA,
	GPIOA,
	GPIOA,
	GPIOC,
	GPIOC,
	GPIOC,
	GPIOC,
	GPIOC,
	GPIOB,
	GPIOB,
	GPIOA,
	GPIOA,
	GPIOA,
	GPIOA,
	GPIOC,
	GPIOC,
	GPIOC,
	GPIOC,
	GPIOB,
	GPIOB,
	GPIOB,
	GPIOB,
	GPIOB,
	GPIOB
};
uint16_t tssp_pin_number[tssp_size] = {
	GPIO_PIN_1,
	GPIO_PIN_0,
	GPIO_PIN_5,
	GPIO_PIN_4,
	GPIO_PIN_7,
	GPIO_PIN_6,
	GPIO_PIN_5,
	GPIO_PIN_4,
	GPIO_PIN_3,
	GPIO_PIN_2,
	GPIO_PIN_1,
	GPIO_PIN_3,
	GPIO_PIN_2,
	GPIO_PIN_1,
	GPIO_PIN_0,
	GPIO_PIN_13,
	GPIO_PIN_9,
	GPIO_PIN_8,
	GPIO_PIN_12,
	GPIO_PIN_11,
	GPIO_PIN_10,
	GPIO_PIN_9,
	GPIO_PIN_9,
	GPIO_PIN_8,
	GPIO_PIN_7,
	GPIO_PIN_6,
	GPIO_PIN_15,
	GPIO_PIN_14,
	GPIO_PIN_13,
	GPIO_PIN_12,
	GPIO_PIN_11,
	GPIO_PIN_10
};

const uint8_t inter_cycles = 102;
float tssp_step = 11.25f * TORAD;			// degree between two tssp in radians

uint8_t tssp_counter[tssp_size];
float tssp_x[tssp_size];
float tssp_y[tssp_size];

uint8_t interpolation_uart[11]; // 4 (float) + 1 (uint8_t) + 4 (float) + 1 (char) + 1 (char) = 11 bytes
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
uint8_t read_interpolation(float *angle, float *distance);
void send_interpolation(uint8_t is_exist, float angle, float distance);
float exp_average_filter(float x);
float constrain_angle(float x);
void float2bytes(uint8_t *a, float val);

float moving_average(float x, float *average, uint16_t *idx, float *buff);
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
	for(uint8_t i = 0; i < tssp_size; ++i)
	{
		tssp_x[i] = cos(tssp_step * (float)i);
		tssp_y[i] = sin(tssp_step * (float)i);
	}
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
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	float angle = 0, distance = 0;
  while (1)
  {		
		uint8_t is_exist = read_interpolation(&angle, &distance);
		send_interpolation(is_exist, angle, distance);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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

  /*Configure GPIO pins : PC13 PC0 PC1 PC2 
                           PC3 PC4 PC5 PC6 
                           PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 
                           PA5 PA6 PA7 PA9 
                           PA10 PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9 
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 
                           PB12 PB13 PB14 PB15 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
uint8_t read_interpolation(float *angle, float *distance)
{
	static float dst_average = 0.0f;
	static uint16_t dst_idx = 0;
	static float dst_buff[15];
	
	for(uint8_t i = 0; i < tssp_size; ++i)
	{
		tssp_counter[i] = 0;
	}
	
	for(uint8_t i = 0; i < inter_cycles; ++i)
	{
		for(uint8_t tssp_i = 0; tssp_i < tssp_size; ++tssp_i)
		{
			tssp_counter[tssp_i] += !(uint8_t)(HAL_GPIO_ReadPin(tssp_pin_base[tssp_i], tssp_pin_number[tssp_i]) == GPIO_PIN_SET);
		}
	}
	
	uint8_t is_exist = 0;
	float x = 0, y = 0;
	for(uint8_t i = 0; i < tssp_size; ++i)
	{
		if(tssp_counter[i] > HIGH_TSSP || tssp_counter[i] < LOW_TSSP)
		{
			tssp_counter[i] = 0;
		}
		
		if(tssp_counter[i] > 0)
		{
			is_exist = 1;
		}
		
		x += tssp_x[i] * (float)tssp_counter[i];
		y += tssp_y[i] * (float)tssp_counter[i];
	}
	(*angle) = constrain_angle(atan2f(y, x) * TODEG); // between (-180; 180]
	
	(*distance) = 0;
	for(uint8_t i = 0; i < tssp_size; ++i)
	{
		if((float)tssp_counter[i] > (*distance))
		{
			(*distance) = (float)tssp_counter[i];
		}
	}
	//(*distance) = exp_average_filter(*distance);
	
	(*distance) = moving_average((*distance), &dst_average, &dst_idx, dst_buff);
	
	return is_exist;
}

void send_interpolation(uint8_t is_exist, float angle, float distance)
{
	interpolation_uart[0] = 97; // a symbol
	interpolation_uart[1] = is_exist;
	float2bytes(interpolation_uart + 2, angle);
	interpolation_uart[6] = 100; // d symbol
	float2bytes(interpolation_uart + 7, distance);
	
	// a(float angle)d(float distance)
	
	while(HAL_UART_Transmit_IT(&huart4, interpolation_uart, 11) == HAL_BUSY);
}

float constrain_angle(float x)
{
	x = fmodf(x + 180.0f, 360.0f);
	if(x < 0)
	{
		x += 360.0f;
	}
	
	return x - 180.0f;
}

float exp_average_filter(float x)
{
  static float filtered_x = 0;
	static float k = 0.7f;
	
  filtered_x += (x - filtered_x) * k;
  return filtered_x;
}

void float2bytes(uint8_t *a, float val)
{
	memcpy(a, &val, 4);
}

float moving_average(float x, float *average, uint16_t *idx, float *buff) {
	static const uint16_t NUM_READ = 15;
	
  if (++(*idx) >= NUM_READ) (*idx) = 0;
  (*average) -= buff[(*idx)];
  (*average) += x;
  buff[(*idx)] = x;
	
  return ((float)(*average) / (float)NUM_READ);
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
