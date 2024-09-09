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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

  typedef enum {case00, case01, case10, case11} estados;
  estados casos;

  uint8_t patronprevio[] = {1, 1, 1, 1, 0, 0, 0, 0};
  uint8_t patronactual[8];

  for (int i=0; i<8; i++){
    		  patronactual[i] = patronprevio [i];
    		  }

  uint8_t aux_sgte[8], aux_pr[8];

  void ToggleIfOn(void) {																	//SI elemento=1 , Toggle it
	  if(patronactual[0] == GPIO_PIN_SET) HAL_GPIO_TogglePin(L1_GPIO_Port, L1_Pin);
	  if(patronactual[1] == GPIO_PIN_SET) HAL_GPIO_TogglePin(L2_GPIO_Port, L2_Pin);
	  if(patronactual[2] == GPIO_PIN_SET) HAL_GPIO_TogglePin(L3_GPIO_Port, L3_Pin);
	  if(patronactual[3] == GPIO_PIN_SET) HAL_GPIO_TogglePin(L4_GPIO_Port, L4_Pin);
	  if(patronactual[4] == GPIO_PIN_SET) HAL_GPIO_TogglePin(L5_GPIO_Port, L5_Pin);
	  if(patronactual[5] == GPIO_PIN_SET) HAL_GPIO_TogglePin(L6_GPIO_Port, L6_Pin);
	  if(patronactual[6] == GPIO_PIN_SET) HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
	  if(patronactual[7] == GPIO_PIN_SET) HAL_GPIO_TogglePin(L8_GPIO_Port, L8_Pin);
  }

  void ShifteoIzq(void){
		  aux_sgte[0] = aux_pr[1];
		  aux_sgte[1] = aux_pr[2];
		  aux_sgte[2] = aux_pr[3];
		  aux_sgte[3] = aux_pr[4];
		  aux_sgte[4] = aux_pr[5];
		  aux_sgte[5] = aux_pr[6];
		  aux_sgte[6] = aux_pr[7];
		  aux_sgte[7] = aux_pr[0];
  }

  void ShifteoDer(void){
	  aux_sgte[1] = aux_pr[0];
	  aux_sgte[2] = aux_pr[1];
	  aux_sgte[3] = aux_pr[2];
	  aux_sgte[4] = aux_pr[3];
	  aux_sgte[5] = aux_pr[4];
	  aux_sgte[6] = aux_pr[5];
	  aux_sgte[7] = aux_pr[6];
	  aux_sgte[0] = aux_pr[7];
  }

  GPIO_PinState SW1, SW2;

  void ReadSWs(void) {												//Leer estado actual de switches
	  SW1 = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
	  SW2 = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
  }


  void ActualizarLEDs(void) { 										//ACTUALIZA LEDs a al patron actual
	  HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, aux_sgte[0]);
	  HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, aux_sgte[1]);
	  HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, aux_sgte[2]);
	  HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, aux_sgte[3]);
	  HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, aux_sgte[4]);
	  HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, aux_sgte[5]);
	  HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, aux_sgte[6]);
	  HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, aux_sgte[7]);
  }
  void TurnOffAllLEDs(void) {
      HAL_GPIO_WritePin(L1_GPIO_Port, L1_Pin, 0);
      HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, 0);
      HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, 0);
      HAL_GPIO_WritePin(L4_GPIO_Port, L4_Pin, 0);
      HAL_GPIO_WritePin(L5_GPIO_Port, L5_Pin, 0);
      HAL_GPIO_WritePin(L6_GPIO_Port, L6_Pin, 0);
      HAL_GPIO_WritePin(L7_GPIO_Port, L7_Pin, 0);
      HAL_GPIO_WritePin(L8_GPIO_Port, L8_Pin, 0);

  }
  void ToggleAllLEDs(void) {
      HAL_GPIO_TogglePin(L1_GPIO_Port, L1_Pin);
      HAL_GPIO_TogglePin(L2_GPIO_Port, L2_Pin);
      HAL_GPIO_TogglePin(L3_GPIO_Port, L3_Pin);
      HAL_GPIO_TogglePin(L4_GPIO_Port, L4_Pin);
      HAL_GPIO_TogglePin(L5_GPIO_Port, L5_Pin);
      HAL_GPIO_TogglePin(L6_GPIO_Port, L6_Pin);
      HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
      HAL_GPIO_TogglePin(L8_GPIO_Port, L8_Pin);

  }
  /*void ToggleAllLEDs(void) {
      HAL_GPIO_TogglePin(L1_GPIO_Port, L1_Pin);
      HAL_GPIO_TogglePin(L2_GPIO_Port, L2_Pin);
      HAL_GPIO_TogglePin(L3_GPIO_Port, L3_Pin);
      HAL_GPIO_TogglePin(L4_GPIO_Port, L4_Pin);
      HAL_GPIO_TogglePin(L5_GPIO_Port, L5_Pin);
      HAL_GPIO_TogglePin(L6_GPIO_Port, L6_Pin);
      HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
      HAL_GPIO_TogglePin(L8_GPIO_Port, L8_Pin);

  } */
  GPIO_PinState botonprevio = GPIO_PIN_SET;
  GPIO_PinState botonactual;
  GPIO_PinState funcionando = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  GPIO_PinState botonactual = HAL_GPIO_ReadPin(BB_GPIO_Port, BB_Pin);

	  if (funcionando == 0 && botonactual == GPIO_PIN_RESET && botonprevio == GPIO_PIN_SET){
		  funcionando = 1;
	  	  HAL_Delay(350);
	  }
	  else if (funcionando == 1 && botonactual == GPIO_PIN_RESET && botonprevio == GPIO_PIN_SET){
	  	  funcionando = 0;
	  	  HAL_Delay(350);
	  }


	  if(funcionando == 1){

		  ReadSWs();

		  if(SW1 == GPIO_PIN_RESET && SW2 == GPIO_PIN_RESET)
			  casos = case00;
		  else if(SW1 == GPIO_PIN_RESET && SW2 == GPIO_PIN_SET)
			  casos = case01;
		  else if(SW1 == GPIO_PIN_SET && SW2 == GPIO_PIN_RESET)
			  casos = case10;
		  else if(SW1 == GPIO_PIN_SET && SW2 == GPIO_PIN_SET)
			  casos = case11;



		  switch(casos){
		  case case00:
			  for (int i=0; i<8; i++){
			  patronactual[i] = patronprevio[i];
			  }
			  ToggleIfOn();
			  HAL_Delay(500);
			  break;


		  case case01:
				  for (int i=0; i<8; i++){
					  patronactual[i] = patronprevio[i];
					  aux_pr[i] = patronactual[i];
				  }

				  ShifteoDer();
				  ActualizarLEDs();
				  for (int i=0; i<8; i++){
					  patronprevio[i] = aux_sgte[i];
				  }
				  HAL_Delay(500);

			  break;
		  case case10:
				  for (int i=0; i<8; i++){
					  patronactual[i] = patronprevio[i];
					  aux_pr[i] = patronactual[i];
				  }

				  ShifteoIzq();
				  ActualizarLEDs();
				  for (int i=0; i<8; i++){
					  patronprevio[i] = aux_sgte[i];
				  }
				  HAL_Delay(500);

			  break;
		  case case11:
			  TurnOffAllLEDs();
			  HAL_Delay(500);
			  ToggleAllLEDs();
			  HAL_Delay(500);
			  TurnOffAllLEDs();
			  break;


		  }
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, L6_Pin|L5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L1_Pin|L2_Pin|L3_Pin|LD2_Pin
                          |L7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L4_Pin|L8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BB_Pin */
  GPIO_InitStruct.Pin = BB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L6_Pin L5_Pin */
  GPIO_InitStruct.Pin = L6_Pin|L5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : L1_Pin L2_Pin L3_Pin LD2_Pin
                           L7_Pin */
  GPIO_InitStruct.Pin = L1_Pin|L2_Pin|L3_Pin|LD2_Pin
                          |L7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L4_Pin L8_Pin */
  GPIO_InitStruct.Pin = L4_Pin|L8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SW1_Pin SW2_Pin */
  GPIO_InitStruct.Pin = SW1_Pin|SW2_Pin;
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
