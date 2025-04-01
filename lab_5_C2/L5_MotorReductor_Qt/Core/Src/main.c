/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "motorDriver.h"
#include "retardos.h"
#include <stdbool.h>


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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */


uint32_t tiempo=0;
uint32_t start_time = 0;
uint32_t stop_time = 0;
uint32_t TIEMPO = 0;
uint32_t contador=0;


int8_t numDatos=0;
uint8_t RPM = 0;
uint8_t rpm = 0;

//unsigned long datosRX[];

volatile uint8_t datosRX[99];
volatile uint8_t indexRX = 0;
uint8_t datosTX[99];
uint8_t indexTX = 0;
//uint16_t overhead;
volatile int32_t posicionMotor =0;
uint8_t datosx [40];

volatile int i;
//volatile int bandera= 0;
volatile bool bandera= 0;
volatile uint8_t TiempoRPM = 0;

//uint32_t numDatos;

typedef struct{
	uint8_t inicio;
	uint8_t tamano;
	uint8_t *datos;  //changeable
	uint8_t crc;
	uint8_t fin;

}PAQUETE;


uint8_t datos4[] = {0xAA,0xFF,0xCC,0xDD,0xCC,0xFF,0xAA,0xFF,0xCC,0xDD,0xCC,0xFF}; //corregir despues max 6 para estudio qt

uint8_t hola[] = {0xEE,0x01};

PAQUETE pk1;

uint8_t calcularCRC(uint8_t *datos, uint8_t tam) //el valor de tam cuando llega
{
	uint8_t crc = 0;
	for(int i=0; i<tam; i++)
	{
		crc ^= datos[i];
	}
	return crc;
}

int8_t serializarPaquete(const PAQUETE* paquete, uint8_t *buffer){   //serialize

	int idx =0;

	if(!paquete || !buffer)return -1;

	buffer[idx++] = paquete->inicio;
	buffer[idx++]= paquete->tamano;

		if(paquete->datos && (paquete->tamano>3)){   	//packet of data ;
		  memcpy(&buffer[idx], paquete->datos, (size_t)(paquete->tamano-4));
		  idx = idx + paquete->tamano-4;
		}

	uint8_t ss= (uint8_t)idx;
	buffer[idx++] =calcularCRC(buffer,ss);
	buffer[idx++] =paquete->fin;
	return idx;
}

void EnviarPaquete(uint8_t *dat, uint8_t tam){ //DAT ES VARIABLE DINAMICA (*dat=RPM)

	pk1.inicio = 0x77; //Start byte
	pk1.tamano = tam + 4;
	pk1.datos=datosx;

    memcpy(&pk1.datos[0], dat, tam + 4 );

	numDatos = serializarPaquete(&pk1, &datosTX);
	pk1.crc=calcularCRC(datosTX, tam+4);
	pk1.fin =0x12;

	CDC_Transmit_FS(datosTX, numDatos);

} //way important to transmit


/*
int8_t deserializaPaquete(const PAQUETE* paquete, uint8_t *buffer){

	ind idx2 =0;
	buffer[idx2]=

}*/

/*
uint8_t ObtenerRPM()
{


	 static uint32_t pos_anterior = 0;
	    uint32_t pos_actual = pos;
	    // Calcula los pulsos transcurridos en 10 segundos
	    uint32_t pulsos_delta = pos_actual - pos_anterior;
	    pos_anterior = pos_actual; // Actualiza para la próxima lectura

	    // Calcula RPM: (pulsos / 1430 pulsos/rev) / 10 seg * 60 seg/min → (pulsos * 6) / 1430
	    uint32_t rpm = (pulsos_delta * 60) / (1430 * 10); // 10 segundos

	    // Asegura que el RPM no exceda 255 (máximo de uint8_t)
	    if (rpm > 255) rpm = 255;

	    return (uint8_t)rpm;

	    /*
    HAL_TIM_Base_Start(&htim3);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    Delay_ms(1000);

    uint32_t TiempoRPM = __HAL_TIM_GET_COUNTER(&htim3);
    uint8_t posimotor = posicionMotor;
       posimotor/=1430;  //constante de una vuelta en pulsos importante

    if (TiempoRPM >= 1000000)
    {

        __HAL_TIM_SET_COUNTER(&htim3, 0);
        float RPMS = (posimotor * 60.0) / TiempoRPM;
        return (uint8_t)RPMS;
        }
} */


uint8_t ObtenerRPM()
{
    static uint32_t pos_anterior = 0;
    uint32_t pos_actual = __HAL_TIM_GET_COUNTER(&htim2);
    uint32_t pulsos_delta = pos_actual - pos_anterior;
    pos_anterior = pos_actual;

   uint32_t rpm = (pulsos_delta * 60) / 1430*10;

    if (rpm > 255) rpm = 255;

    return (uint8_t)rpm;
}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */



void CDC_ReceiveCallBack(uint8_t* Buf, uint32_t len){

	memcpy(datosRX,Buf,len);
	indexRX = len;
}


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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  //ssd1306_Init();
  motoresInit(&htim2, TIM_CHANNEL_1, TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim3);

  delay_us_tim_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  //importante por encoder mode t1, t2.


  while (1)
  {



  	//CDC_Transmit_FS(&contador,1);

/*    HAL_Delay(1);
	//  contador++;


	  if (__HAL_TIM_GET_COUNTER(&htim3)>1000000)
	  {
		  posicionMotor = __HAL_TIM_GET_COUNTER(&htim2);
		  RPM = ObtenerRPM(posicionMotor);
		  //RPM SE ENVIA A QT
		  //EnviarPaquete(RPM, 1);
		  __HAL_TIM_SET_COUNTER(&htim3,0);
	  }*/

/*
	   if(indexRX !=0){
			  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	   }   */

	     posicionMotor = __HAL_TIM_GET_COUNTER(&htim2);

	    // ObtenerRPM(posicionMotor);

		   if(!bandera){
		   	       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
		   	       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
		   	       EnviarPaquete(datos4, 12);
		   	      bandera= 1;
		   }
		    // HAL_Delay(1);
		     Delay_ms(1);


	   if(tiempo++ == 10000){
		   tiempo=0;

			  RPM = ObtenerRPM();

   	       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
   	       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
		     Delay_ms(2);
  	       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 1);
   	       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 1);
   	          Delay_ms(500);
  	       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, 0);
   	       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);


		   /*
            char texto[64];
            sprintf (texto, "&d\n", posicionMotor);
            CDC_Transmit_FS(texto, strlen(texto));
           */
		   /*
		   datosTX[0]=0xF0;
	       datosTX[1]= 0X08;
		   datosTX[2]=((contador >> 24)&0xFF);
		   datosTX[3]=((contador >> 16)&0xFF);
		   datosTX[4]=((contador >> 8)&0xFF);
		   datosTX[5]=((contador >> 0)&0xFF);
		   datosTX[6]= 0x23;
		   datosTX[7]= 0x03;
           CDC_Transmit_FS(datosTX, 8);*/

			 // EnviarPaquete(hola, 2);
			   EnviarPaquete(hola, 2);
	      }


  /*	sprintf(texto1, "el contador va en %lu \n",contador);
  	CDC_Transmit_FS(texto1, strlen(texto1));
  	contador++;
	HAL_Delay(200); */

	  // ssd1306

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 96-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 96-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //milisegundos++;
}


/*
void CDC_ReceiveCallBack(uint8_t* Buf, uint32_t Len){
	mempcy(datosRX,Buf,Len);
	indexRX = Len;
}*/

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
