/*
 * my_it_callbacks.c
 *
 *  Created on: Sep 21, 2024
 *      Author: lbast
 */


#include "main.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/////////////////////////////////////////////
uint8_t bloqueado=0;
uint8_t contador=0;
uint32_t muestras_totales;

extern uint8_t rxData;
char txData[50];
//////////////////// A D C ////////////////
float voltage[100];
uint32_t adcValue;
////////////////////////////////////////////
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;


uint32_t numero_muestras(uint32_t frecuencia_hz, uint32_t duracion_seg) {
    float periodo = 1.0f / (float)frecuencia_hz;
    uint32_t numerodemuestras = (uint32_t)(duracion_seg / periodo);
    return numerodemuestras;
}

/////////////// I N T E R R U P T  U A R T  RX /////////////////////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2){
    	if(bloqueado){
    		HAL_UART_Transmit_IT(&huart2, (uint8_t*)"\n BUSY \n", strlen("\n BUSY \n"));
    	}

    	else{
    		if(rxData == 'A'){
    			__HAL_TIM_SET_AUTORELOAD(&htim3, 39); //Set clock to 50hz
    			muestras_totales = numero_muestras(50, 1);
    			HAL_TIM_Base_Start_IT(&htim3);
    		}

    		else if(rxData=='B'){
    			__HAL_TIM_SET_AUTORELOAD(&htim3, 39); //Set clock to 50hz
    			muestras_totales = numero_muestras(50, 2);
    			HAL_TIM_Base_Start_IT(&htim3);
    		}

    		else if(rxData=='C'){
    			__HAL_TIM_SET_AUTORELOAD(&htim3, 19); //Set clock to 100hz
    			muestras_totales = numero_muestras(100, 1);
    			HAL_TIM_Base_Start_IT(&htim3);
    		}

    		else if(rxData=='D'){
    			__HAL_TIM_SET_AUTORELOAD(&htim3, 19); //Set clock to 100hz
    			muestras_totales = numero_muestras(100, 2);
    			HAL_TIM_Base_Start_IT(&htim3);
    		}

    		else{
    		    HAL_UART_Transmit_IT(&huart2, (uint8_t*)"\nComando Desconocido\n", strlen("\nComando Desconocido\n"));
    		}

    	}

        HAL_UART_Receive_IT(&huart2, &rxData, 1);  // Reinicia la recepción para el siguiente byte
    }
}




//////////////////////////////////// I N T E R R U P T   T I M E R /////////////////////////////////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
////////////////////// I N T E R R U P T   T I M E R  O C U P A D O ////////////////////////////////////
    if (htim->Instance == TIM2)  // CONDICIONAL QUE VERIFICA CUAL TIMER GENERO LA INTERRUPCIÓN
    {
    	if(bloqueado == 1)
    		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    	else
    		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
    }

///////////////////// I N T E R R U P T   T I M E R   M U E S T R E O //////////////////////////////////
    if (htim->Instance == TIM3)
    {
    	if (contador>muestras_totales){
    			bloqueado=0;
    			HAL_TIM_Base_Stop_IT(&htim3);
    			contador=0;
    		}
    		else if(contador<=muestras_totales){
    			bloqueado=1;
    			HAL_ADC_Start(&hadc1);
    			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    			adcValue = HAL_ADC_GetValue(&hadc1);
    			voltage[contador] = ((float)adcValue) * 3.3 / 4095;  // 3.3V referencia, 12 bits resolución (4095)
    	        snprintf(txData, sizeof(txData), "Voltaje: %.2f V, Timestamp: %lu ms\r\n", voltage[contador], HAL_GetTick());
    	        HAL_UART_Transmit_IT(&huart2, (uint8_t*)txData, strlen(txData));

    	        contador++;
    		}

    }
}
