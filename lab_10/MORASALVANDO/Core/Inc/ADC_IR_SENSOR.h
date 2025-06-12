/*
 * ADC_IR_SENSOR.h
 *
 *  Created on: May 3, 2025
 *      Author: Yeison Lara
 */


/*
 *
 *  *ADC = address of the peripheral (ADC)
 *  *TIM = address of the timmer (delays)
 *  ADC_CHANNELS[5] = ADC channels that will do the conversion of the receiving LEDs
 *  IR_PORTS[5] = physical pins where the emitting LEDs are located
 *  IR_PINS[5] = pins to activate the emitting LEDs
 *  *outArr = array that receives the conversions
 *  samples = times as many times as you need to average
 *
 * */


/*
#ifndef ADC_IR_SENSOR_H_
#define ADC_IR_SENSOR_H_

#include "main.h"
#include <math.h>

typedef struct {
    float DIST1;
    float DIST2;
    float DIST3;
    float DIST4;
    float BATT;
} IR_ConversionResult;

void CONVERTER_INIT(ADC_HandleTypeDef *ADCUSER, TIM_HandleTypeDef *TIM, uint32_t ADC_CHANNELS[4], GPIO_TypeDef* IR_PORTS[4], uint16_t IR_PINS[4]);
void ReadAndAverageSensors( uint16_t *outArr, uint8_t samples);

IR_ConversionResult CONV_Y_TRANS(void);

#endif /* ADC_IR_SENSOR_H_ */

