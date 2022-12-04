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
#include "stdio.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SCREEN_ORIENTATION 1 //0: Retrato | 1: Paisagem
#define SCREEN_BG BLACK //Cor de fundo

/* Mais cores (em RGB 565) */
#define GREY 0xC618
#define ORANGE 0xF9E0
#define PURPLE 0x68FF

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void setScreenSettings(void);
void drawScenery(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t b2Flag = 0;
uint32_t deb = 0;
uint32_t dbgx = 0, dbgy = 0;
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  setScreenSettings();

  //Ângulos: 0, 15, 30, 45, 60, 75, 90
  int16_t angles[7][2] = {{250, 230}, {252, 216}, {257, 203}, {266, 191}, {278, 182}, {291, 177}, {305, 175}};
  uint8_t st = 0; //Etapa
  int16_t xball = 305, yball = 220; //Posição da bola
  int16_t spd = 0, ang = 0, xspd = 0, yspd = 0; //Parâmetros
  char cspd[50]; //String para debug

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  switch(st)
	  {
	  	  case 0: //Início
	  		  drawScenery(); //Desenha o cenário
	  		  xball = 305;
	  		  yball = 220;
	  		  fillCircle(xball, yball, 10, ORANGE); //Desenha bola no canto
	  		  st = 1;
	  		  break;

	  	  case 1: //Selecionar ângulo (0 a 90 graus)
	  		  ang = -15;
	  		  for(uint16_t i = 0; i < 7; i++)
	  		  {
	  			  ang += 15;
	  			  drawLine(305, 230, angles[i][0], angles[i][1], RED); //Desenha
	  			  HAL_Delay(250);
	  			  drawLine(305, 230, angles[i][0], angles[i][1], SCREEN_BG); //Apaga

	  			  if(b2Flag) //Se o botão for pressionado
	  			  {
	  				  drawLine(305, 230, angles[i][0], angles[i][1], RED); //Re-desenha
	  				  i = 8; //Sai do loop
	  				  st = 2;
	  				  b2Flag = 0;
	  				  sprintf(cspd, "ang = %d\r\n", ang);
	  				  HAL_UART_Transmit(&huart2, (uint8_t *)&cspd, 10, 100);
	  			  }
	  		  }
	  		  break;

	  	  case 2: //Selecionar velocidade de arremeço (0 a 140)
	  		  spd = -1;
	  		  fillRect(20, 17, 280, 11, SCREEN_BG); //Apaga toda a barra

	  		  for(uint16_t i = 0; i < 140; i++) //barra com 280px = 140 intensidades possíveis
	  		  {
	  			  spd++;
	  			  fillRect(20+(2*spd), 17, 2, 11, YELLOW); //Desenha
	  			  HAL_Delay(50);

	  			  if(b2Flag) //Se o botão for pressionado
	  			  {
	  				  i = 280; //Sai do loop
	  				  st = 3;
	  				  b2Flag = 0;
	  				  sprintf(cspd, "spd = %d\r\n", spd);
	  				  HAL_UART_Transmit(&huart2, (uint8_t *)&cspd, 10, 100);
	  			  }
	  		  }
	  		  break;

	  	  case 3: //Decompõe a velocidade em x e y de acordo com o ângulo
	  		  xspd = (int16_t)(cos((ang*M_PI)/180)*(spd));
	  		  yspd = (int16_t)(sin((ang*M_PI)/180)*(spd));

	  		  dbgx = xspd;
	  		  dbgy = yspd;

	  		  st = 4;

	  		  break;

	  	  case 4: //Movimenta bola

	  		  while(xball < 319 && yball < 239)
	  		  {
	  			  fillCircle(xball, yball, 10, SCREEN_BG); //Apaga
	  			  xball -= xspd;
	  			  yball -= yspd;
	  			  fillCircle(xball, yball, 10, ORANGE); //Desenha
	  			  HAL_Delay(100);
	  			  yspd -= 5; //"gravidade"
	  		  }
	  		  fillCircle(xball, yball, 10, SCREEN_BG); //Apaga
	  		  st = 0;
	  		  break;
	  }

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

  /*Configure GPIO pin : B2_Pin */
  GPIO_InitStruct.Pin = B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/**
 * @brief Configura o LCD
 */
void setScreenSettings(void)
{
	tft_gpio_init(); //Inicializa os GPIOs do LCD (evita uso do CubeMX)
	HAL_TIM_Base_Start(&htim1); //Inicializa o Timer1 (base de tempo de us do LCD)
	uint16_t ID = tft_readID(); //Lê o ID do LCD (poderia ser chamada pela inicialização do LCD)
	HAL_Delay(100);
	tft_init(ID); //Inicializa o LCD de acordo com seu ID
	setRotation(SCREEN_ORIENTATION); //Ajusta a orientação da tela (paisagem)
	fillScreen(SCREEN_BG);
}

/**
 * @brief Sobrescreve função de Callback da interrupção de GPIO
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == B2_Pin && HAL_GetTick() > deb+50)
	{
		b2Flag = 1;
		deb = HAL_GetTick();
	}
}

/**
 * @brief Desenha o cenário
 */
void drawScenery(void)
{
	fillScreen(SCREEN_BG);
	drawLine(15, 105, 15, 230, WHITE); //Poste
	drawLine(15, 105, 25, 80, WHITE); //Diagonal
	drawLine(25, 60, 25, 125, WHITE); //Quadro
	drawLine(30, 80, 30, 115, WHITE); //L vertical
	drawLine(30, 115, 65, 115, WHITE); //L horizontal
	drawLine(33, 115, 37, 140, GREY); //Cesta '\'
	drawLine(62, 115, 57, 140, GREY); //Cesta '/'
	drawLine(37, 140, 57, 140, GREY); //Cesta _
	drawRect(15, 15, 290, 15, WHITE); //Barra x
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
