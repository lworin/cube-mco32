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
#include "fonts.h"
#include "tft.h"
#include "user_setting.h"
#include "functions.h"
#include <stdio.h>
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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t ID = 0;
uint8_t flagSelect = 0;
uint8_t flagBack = 0;
unsigned char cState, pState = 100;
int encoder = 0;
uint8_t input = 0;
uint16_t mem[15];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void drawScreen(uint16_t tela, uint16_t x, uint16_t y);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Representa um estado da FSM
struct State {
	uint16_t out[3];
	unsigned char next[5]; //Vetor de próximos estados
};
typedef const struct State tipoS;

//Apelidos para referenciar os estados da FSM
#define A 0
#define B 1
#define C 2

#define A1 3
#define A2 4
#define A3 5

#define B1 6
#define B2 7
#define B3 8

#define C1 9
#define C2 10
#define C3 11

#define C3X 12
#define C3Y 13
#define C3Z 14

//Estrutura de dados que corresponde ao diagrama de transição de estados da FSM
tipoS Fsm[15] = {/*	Tela, x, y			0		1		2		3		4		*/
					{{1, 20,  40}, {	A,		B,		C,		A1,		A	}},	//Estado A
					{{1, 20, 120}, {	B, 		C,		A,		B1,		B	}},	//Estado B
					{{1, 20, 200}, {	C, 		A,		B,		C1,		C	}},	//Estado C

					{{2, 20,  40}, {	A1, 	A2,		A3,		A1,		A	}},	//Estado A1
					{{2, 20, 120}, {	A2, 	A3,		A1,		A2,		A	}},	//Estado A2
					{{2, 20, 200}, {	A3, 	A1,		A2,		A3,		A	}},	//Estado A3

					{{3, 20,  40}, {	B1, 	B2,		B3,		B1,		B	}},	//Estado B1
					{{3, 20, 120}, {	B2, 	B3,		B1,		B2,		B	}},	//Estado B2
					{{3, 20, 200}, {	B3, 	B1,		B2,		B3,		B	}},	//Estado B3

					{{4, 20,  40}, {	C1, 	C2,		C3,		C1,		C	}},	//Estado C1
					{{4, 20, 120}, {	C2, 	C3,		C1,		C2,		C	}},	//Estado C2
					{{4, 20, 200}, {	C3, 	C1,		C2,		C3X,	C	}},	//Estado C3

					{{5, 20,  40}, {	C3X, 	C3Y,	C3Z,	C3X,	C3	}},	//Estado C3X
					{{5, 20, 120}, {	C3Y, 	C3Z,	C3X,	C3Y,	C3	}},	//Estado C3Y
					{{5, 20, 200}, {	C3Z, 	C3X,	C3Y,	C3Z,	C3	}}	//Estado C3Z
};



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
  //tft_gpio_init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /* Sequência de inicialização do LCD */
  tft_gpio_init(); //Inicializa os GPIOs do LCD (evita uso do CubeMX)
  HAL_TIM_Base_Start(&htim1); //Inicializa o Timer1 (base de tempo de us do LCD)
  ID = tft_readID(); //Lê o ID do LCD (poderia ser chamada pela inicialização do LCD)
  HAL_Delay(100);
  tft_init(ID); //Inicializa o LCD de acordo com seu ID
  setRotation(0); //Ajusta a orientação da tela (retrato)
  fillScreen(BLACK); //Preenche a tela em preto

  cState = A; //Inicia no estado A

  //Define todos os itens como não selecionados
  for(int i = A; i<=C3Z; i++)
	  mem[i] = WHITE;

  //Habilita o encoder
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  //Zera o contador
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  //Atualiza variável

  encoder = __HAL_TIM_GET_COUNTER(&htim3)>>1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* 1. Saída baseada no estado atual */
	  if((cState != pState) || flagSelect == 2)
	  {
		  drawScreen(Fsm[cState].out[0], Fsm[cState].out[1], Fsm[cState].out[2]);
		  pState = cState;
		  flagSelect = 0;
	  }

	  /* 2. Aguarda o tempo predefinido para o estado */

	  //HAL_Delay(500);

	  /* 3. Lê as entradas */
	  if(flagBack == 1)
	  {
		  input = 4;
		  flagBack = 0;
	  }
	  else if(flagSelect == 1)
	  {
		  input = 3;
		  flagSelect = 2;

		  if((cState >= A1)&&(cState <= A3))
		  {
			  mem[A1] = WHITE;
			  mem[A2] = WHITE;
			  mem[A3] = WHITE;
		  }
		  else if((cState >= B1)&&(cState <= B3))
		  {
			  mem[B1] = WHITE;
			  mem[B2] = WHITE;
			  mem[B3] = WHITE;
		  }
		  else if((cState >= C1)&&(cState <= C2))
		  {
			  mem[C1] = WHITE;
			  mem[C2] = WHITE;
		  }
		  else if((cState >= C3X)&&(cState <= C3Z))
		  {
			  mem[C3X] = WHITE;
			  mem[C3Y] = WHITE;
			  mem[C3Z] = WHITE;
		  }

		  mem[cState] = RED;
	  }
	  else if(encoder > __HAL_TIM_GET_COUNTER(&htim3)>>1)
	  {
		  input = 2;
		  encoder = __HAL_TIM_GET_COUNTER(&htim3)>>1;
	  }
	  else if(encoder < __HAL_TIM_GET_COUNTER(&htim3)>>1)
	  {
		  input = 1;
		  encoder = __HAL_TIM_GET_COUNTER(&htim3)>>1;
	  }
	  else
	  {
		  input = 0;
	  }

	  /* 4. Vai para o próximo estado, que depende da entrada e do estado atual */
	  cState = Fsm[cState].next[input];

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC1 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 LD2_Pin
                           PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pins : btSelect_Pin btBack_Pin */
  GPIO_InitStruct.Pin = btSelect_Pin|btBack_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */

/**
 * @brief Desenha a tela
 */
void drawScreen(uint16_t tela, uint16_t x, uint16_t y)
{
	uint8_t titulo[10];
	sprintf((char*) titulo, "Tela %d", tela);
	//fillScreen(BLACK); //Preenche a tela em preto
	printnewtstr_bc(15, GREEN, BLACK, &mono12x7bold, 1, titulo);

	switch(tela)
	{
	case 1:
		printnewtstr_bc(70, WHITE, BLACK, &mono12x7bold, 1, (uint8_t*)"   A  "); //Item 1
		drawRoundRect(20, 40, 200, 50, 10, WHITE); //Item 1
		printnewtstr_bc(150, WHITE, BLACK, &mono12x7bold, 1, (uint8_t*)"   B  "); //Item 2
		drawRoundRect(20, 120, 200, 50, 10, WHITE); //Item 2
		printnewtstr_bc(230, WHITE, BLACK, &mono12x7bold, 1, (uint8_t*)"   C  "); //Item 3
		drawRoundRect(20, 200, 200, 50, 10, WHITE); //Item 3
		break;

	case 2:
		printnewtstr_bc(70, mem[A1], BLACK, &mono12x7bold, 1, (uint8_t*)"   A1 "); //Item 1
		drawRoundRect(20, 40, 200, 50, 10, WHITE); //Item 1
		printnewtstr_bc(150, mem[A2], BLACK, &mono12x7bold, 1, (uint8_t*)"   A2 "); //Item 2
		drawRoundRect(20, 120, 200, 50, 10, WHITE); //Item 2
		printnewtstr_bc(230, mem[A3], BLACK, &mono12x7bold, 1, (uint8_t*)"   A3 "); //Item 3
		drawRoundRect(20, 200, 200, 50, 10, WHITE); //Item 3
		break;

	case 3:
		printnewtstr_bc(70, mem[B1], BLACK, &mono12x7bold, 1, (uint8_t*)"   B1 "); //Item 1
		drawRoundRect(20, 40, 200, 50, 10, WHITE); //Item 1
		printnewtstr_bc(150, mem[B2], BLACK, &mono12x7bold, 1, (uint8_t*)"   B2 "); //Item 2
		drawRoundRect(20, 120, 200, 50, 10, WHITE); //Item 2
		printnewtstr_bc(230, mem[B3], BLACK, &mono12x7bold, 1, (uint8_t*)"   B3 "); //Item 3
		drawRoundRect(20, 200, 200, 50, 10, WHITE); //Item 3
		break;

	case 4:
		printnewtstr_bc(70, mem[C1], BLACK, &mono12x7bold, 1, (uint8_t*)"   C1 "); //Item 1
		drawRoundRect(20, 40, 200, 50, 10, WHITE); //Item 1
		printnewtstr_bc(150, mem[C2], BLACK, &mono12x7bold, 1, (uint8_t*)"   C2 "); //Item 2
		drawRoundRect(20, 120, 200, 50, 10, WHITE); //Item 2
		printnewtstr_bc(230, WHITE, BLACK, &mono12x7bold, 1, (uint8_t*)"   C3 "); //Item 3
		drawRoundRect(20, 200, 200, 50, 10, WHITE); //Item 3
		break;

	case 5:
		printnewtstr_bc(70, mem[C3X], BLACK, &mono12x7bold, 1, (uint8_t*)"   C3X"); //Item 1
		drawRoundRect(20, 40, 200, 50, 10, WHITE); //Item 1
		printnewtstr_bc(150, mem[C3Y], BLACK, &mono12x7bold, 1, (uint8_t*)"   C3Y"); //Item 2
		drawRoundRect(20, 120, 200, 50, 10, WHITE); //Item 2
		printnewtstr_bc(230, mem[C3Z], BLACK, &mono12x7bold, 1, (uint8_t*)"   C3Z"); //Item 3
		drawRoundRect(20, 200, 200, 50, 10, WHITE); //Item 3
		break;
	}

	drawRoundRect(x, y, 200, 50, 10, RED); //Cursor
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_10)
	{
		flagSelect = 1;
	}
	else if(GPIO_Pin == GPIO_PIN_11)
	{
		flagBack = 1;
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
