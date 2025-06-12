/*
 * motorDriver.c
 *
 *  Created on: Mar 6, 2025
 *      Author: CamiloA
 */

#include "motorDriver.h"



TIM_HandleTypeDef *timmotorGlobal;


void motoresInit(TIM_HandleTypeDef *timmotor, uint32_t canal1, uint32_t canal2){
	timmotorGlobal = timmotor;
	HAL_TIM_PWM_Start(timmotorGlobal, canal1);   // OUT PWMA
	HAL_TIM_PWM_Start(timmotorGlobal, canal2);  // OUT PWMB
	timmotorGlobal->Instance->CCR1 = 0;
	timmotorGlobal->Instance->CCR2 = 0;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,0);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,0);
}
void motores(int8_t m1, int8_t m2){

	//control motor 1
	if(m1 > 0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,0);
		if(m1 > 100)m1 = 100;
		timmotorGlobal->Instance->CCR1 = m1;
	}else if(m1 < 0){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,1);
		m1 *= -1;
		if(m1 > 100)m1 = 100;
		timmotorGlobal->Instance->CCR1 = m1;
	}else{                                       /// NORMALMENTE SOLO EJECUTA EN 0,0 FREE
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,0);
		timmotorGlobal->Instance->CCR1 = 0;
	}

	//control motor 2
	if(m2 > 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,1);  //COMANDO ADELANTE
		HAL_GPIO_WritePin(GPIOB	, GPIO_PIN_10,0);
		if(m2 > 100)m2 = 100;
		timmotorGlobal->Instance->CCR2 = m2;
	}else if(m2 < 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,0);   //COMANDO ATRAS
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,1);
		m2 *= -1;
		if(m2 > 100)m2 = 100;
		timmotorGlobal->Instance->CCR2 = m2;
	}else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,0);
		timmotorGlobal->Instance->CCR2 = 0;
	}

}
