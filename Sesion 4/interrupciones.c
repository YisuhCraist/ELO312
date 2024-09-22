
#include "main.h"
#include "gpio.h"
#include <stdio.h>
#include "usart.h"

int contador =0;
int funcionando = 0;
int numero_aleatorio;
int cambio;
extern int patronactual[8];

int _write(int le, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, 1000);
	return len;
}
/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)  // Verifica si el callback corresponde al Timer 6
    {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);  // Cambia el estado del pin del LED
    }
    if (htim->Instance == TIM6)  // Verifica si el callback corresponde al Timer 6
    {
    	int ran= rand(contador);
        setvbuf(stdout, NULL, _IONBF, 0);
        printf("La cuenta va en:%d\n",contador);
        printf("Random:%d\n",ran);
        contador = contador+1;// Cambia el estado del pin del LED
    }
}*/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == BB_Pin) // Si el pin que generó la interrupción es el PIN 0
	{
		funcionando = 1;
	}
	if (GPIO_Pin == BOTON_Pin) // Si el pin que generó la interrupción es el PIN 0
	{
		cambio =1;
	}
}
