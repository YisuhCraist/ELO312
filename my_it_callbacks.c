/*
 * my_it_callbacks.c
 *
 *  Created on: Sep 21, 2024
 *      Author: lbast
 */


#include "main.h"
#include <stdlib.h>
#include <string.h>
/////////////////////////////////////////////
extern int bloqueado;
extern uint8_t rxData;
extern char txData;
//////////////////// A D C ////////////////
extern float voltage[100];
extern uint32_t adcValue;
////////////////////////////////////////////
extern TIM_HandleTypeDef htim3;
extern ADC_HandleTypeDef hadc1;
extern UART_HandleTypeDef huart2;
/////////////////////////////////////////////
int start_time;

void adc_config(uint32_t frecuencia_hz, uint32_t duracion_seg){
	uint32_t muestraactual=0;
	float periodo = 1.0f / (float)frecuencia_hz;
	uint32_t numerodemuestras = (uint32_t)duracion_seg/periodo;

	bloqueado=1;

	for (int i; muestraactual<numerodemuestras; i++){
		bloqueado=1;
		HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        adcValue = HAL_ADC_GetValue(&hadc1);
        voltage[i] = ((float)adcValue) * 3.3 / 4095;  // 3.3V referencia, 12 bits resolución (4095)
        snprintf(txData, sizeof(txData), "Voltaje: %.2f V, Timestamp: %lu ms\r\n", voltage[i], HAL_GetTick());
        HAL_UART_Transmit(&huart2, (uint8_t*)txData, strlen(txData), HAL_MAX_DELAY);
	}

	bloqueado=0;
}





void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)  // CONDICIONAL QUE VERIFICA CUAL TIMER GENERO LA INTERRUPCIÓN
    {
    	if(bloqueado == 1)
    		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    	else
    		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
    }

    if (htim->Instance == TIM3)  // TIMER QUE CONTROLA TIEMPO DE REGISTRO Y FRECUENCIA
    {

    	if(rxData == 'A'){

        	adc_config(50, 1);

    	}
    	else if(rxData=='B'){
        	adc_config(50, 2);
    	}
    	else if(rxData=='C'){
        	adc_config(100, 1);
    	}
    	else if(rxData=='D'){
    		if(ejecucion=1)
    			//COSAS DEL ADC
    		else

    		start_time=HAL_GetTick();			//ms de inicio

    			(HAL_GetTick()-start_time) == 2000)

        	adc_config(100, 2);
    	}



    }
}
