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
#include "usart.h"
#include "gpio.h"
#include <stdlib.h>

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

/* USER CODE BEGIN PV */
extern int funcionando;
extern int cambio;
int numero;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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

	int patronprevio[] = {1, 1, 1, 1, 0, 0, 0, 0};
	int patronactual[8];

	for (int i=0; i<8; i++){
		patronactual[i] = patronprevio [i];
	}

	int aux_sgte[8], aux_pr[8];

	void ToggleIfOn(void) {																	//SI elemento=1 , Toggle it
		if(patronactual[0] == 1) HAL_GPIO_TogglePin(L1_GPIO_Port, L1_Pin);
		if(patronactual[1] == 1) HAL_GPIO_TogglePin(L2_GPIO_Port, L2_Pin);
		if(patronactual[2] == 1) HAL_GPIO_TogglePin(L3_GPIO_Port, L3_Pin);
		if(patronactual[3] == 1) HAL_GPIO_TogglePin(L4_GPIO_Port, L4_Pin);
		if(patronactual[4] == 1) HAL_GPIO_TogglePin(L5_GPIO_Port, L5_Pin);
		if(patronactual[5] == 1) HAL_GPIO_TogglePin(L6_GPIO_Port, L6_Pin);
		if(patronactual[6] == 1) HAL_GPIO_TogglePin(L7_GPIO_Port, L7_Pin);
		if(patronactual[7] == 1) HAL_GPIO_TogglePin(L8_GPIO_Port, L8_Pin);
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

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		numero = rand()%8;
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

				if(cambio == 0)
				{
					for (int i=0; i<8; i++)
					{
						patronactual[i] = patronprevio[i];
					}
				}
				if (cambio == 1)
				{
					if(patronactual[numero] == 1)
					{
						patronprevio[numero] = 0;
						cambio =0;
					}else{

						patronprevio[numero] = 1;
						cambio =0;
					}
				}
				ToggleIfOn();
				HAL_Delay(250);

				break;


			case case01:
				if (cambio == 1)
				{
					if(patronactual[numero] == 1)
					{
						patronprevio[numero] = 0;
						cambio =0;
					}else{

						patronprevio[numero] = 1;
						cambio =0;
					}
				}
				if (cambio == 0)
				{
					for (int i=0; i<8; i++){
						patronactual[i] = patronprevio[i];
						aux_pr[i] = patronactual[i];
					}
				}
				ShifteoDer();
				ActualizarLEDs();
				for (int i=0; i<8; i++){
					patronprevio[i] = aux_sgte[i];
				}
				HAL_Delay(500);

				break;
			case case10:

				if(cambio == 1)
				{

					if(patronprevio[numero] ==0)
					{
						patronprevio[numero] = 1;
						cambio = 0;
					}
					if(patronprevio[numero] ==1)
					{
						patronprevio[numero] =0;
						cambio = 0;
					}

				}

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
