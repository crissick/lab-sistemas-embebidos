/*
 * ADC_IR_SENSOR.c
 *
 *  Created on: May 3, 2025
 *      Author: putom
 */

/*


#include "ADC_IR_SENSOR.h"

uint16_t avgReadingsGLOBAL[4];
uint32_t sum = 0;
uint8_t idx;


ADC_HandleTypeDef *ADCUSERGLOBAL;


TIM_HandleTypeDef *TIMGLOBAL;

uint32_t ADC_CHANNELSGLOBAL[4];

GPIO_TypeDef* IR_PORTSGLOBAL[4];

uint16_t IR_PINSGLOBAL[4];

const int8_t mIR1 = -43.62;
const int8_t mIR2 = -51.51;
const int8_t mIR3 = -57.84;
const int8_t mIR4 = -60.11;

IR_ConversionResult CONVS;


void CONVERTER_INIT(ADC_HandleTypeDef *ADCUSER, TIM_HandleTypeDef *TIM, uint32_t ADC_CHANNELS[4], GPIO_TypeDef* IR_PORTS[4], uint16_t IR_PINS[4])
{
	 ADCUSERGLOBAL = ADCUSER;
	 TIMGLOBAL = TIM;
	 for (uint8_t i = 0; i < 4; i++) {
	     ADC_CHANNELSGLOBAL[i] = ADC_CHANNELS[i];
	     IR_PORTSGLOBAL[i]     = IR_PORTS[i];
	     IR_PINSGLOBAL[i]      = IR_PINS[i];
	 }
}

void ReadAndAverageSensors(uint16_t *outArr, uint8_t samples)
{
    for (idx = 0; idx < 4; idx++)
    {
        sum = 0;
        ADC_ChannelConfTypeDef sConfig = {0};
        sConfig.Channel = ADC_CHANNELSGLOBAL[idx];
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

        HAL_ADC_ConfigChannel(ADCUSERGLOBAL, &sConfig);

        for (uint8_t i = 0; i < samples; i++)
        {
        	HAL_GPIO_WritePin(IR_PORTSGLOBAL[idx], IR_PINSGLOBAL[idx], GPIO_PIN_SET);
            __HAL_TIM_SET_COUNTER(TIMGLOBAL,0);

            while (__HAL_TIM_GET_COUNTER(TIMGLOBAL) < 10);
            HAL_ADC_Start(ADCUSERGLOBAL);

            HAL_ADC_PollForConversion(ADCUSERGLOBAL, 10);
            sum += HAL_ADC_GetValue(ADCUSERGLOBAL);
            HAL_ADC_Stop(ADCUSERGLOBAL);
            HAL_GPIO_WritePin(IR_PORTSGLOBAL[idx], IR_PINSGLOBAL[idx], GPIO_PIN_RESET);
        }
        outArr[idx] = sum / samples; avgReadingsGLOBAL[idx] = outArr[idx];
    }
}

IR_ConversionResult CONV_Y_TRANS(void)
{
    CONVS.DIST1 = mIR1 * log(avgReadingsGLOBAL[0]) + 373.25; if (CONVS.DIST1>255)CONVS.DIST1 = 255;
    CONVS.DIST2 = mIR2 * log(avgReadingsGLOBAL[1]) + 439.42; if (CONVS.DIST2>255)CONVS.DIST2 = 255;
    CONVS.DIST3 = mIR3 * log(avgReadingsGLOBAL[2]) + 494.28; if (CONVS.DIST3>255)CONVS.DIST3 = 255;
    CONVS.DIST4 = mIR4 * log(avgReadingsGLOBAL[3]) + 488.57; if (CONVS.DIST4>255)CONVS.DIST4 = 255;

    return CONVS;
}

*/

