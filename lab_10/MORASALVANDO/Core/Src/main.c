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
#include "usbd_cdc_if.h"
#include "stm32f4xx_hal.h"
#include "math.h"

#include "mms_odometria.h"
#include "ADC_IR_SENSOR2.h"




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
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
float DIST_ACUM = 0;
float VEL_FINAL = 0;
float mm_FINAL = 0;
float DIS_ACCE = 0;
float DIS_DESACCE = 0;
float AceleracionGlobal = 0;
float v_desired = 0;


float DIST_ACUM_L = 0;
float VEL_FINAL_L = 0;
float mm_FINAL_L = 0;
float DIS_ACCE_L = 0;
float DIS_DESACCE_L = 0;
float AceleracionGlobal_L = 0;
float v_desired_L = 0;

float velMotores = 0;

//static float v_desired   = 0.0f;

int32_t VELOCIDAD_BOT_REAL;
int32_t pwmTrapecio = 0;

uint32_t canales[4] = {ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3};
GPIO_TypeDef* puertos_led[4] = {GPIOB, GPIOB, GPIOA, GPIOA};
uint16_t pines_led[4] = {GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_8, GPIO_PIN_9};
uint16_t avgReadings[4];

IR_ConversionResult TRANS_CONVS;



int32_t PUNTOS_X;
int32_t PUNTOS_Y;
int32_t DIS_MOTOR_L;
int32_t DIS_MOTOR_R;
float THETA;
int32_t VEL_MOTOR_L;
int32_t VEL_MOTOR_R;
int32_t VEL_LINEAL;
int32_t VEL_ANGULAR;
int32_t AVANCE_LINEA;
int32_t LEFT_DISTANCE_MM_ACUM;
int32_t RIGHT_DISTANCE_MM_ACUM;

Pose ps;


uint32_t  tiempo=0;

volatile uint16_t adcValue[5] = {0};
char texto [50];

int8_t numDatos=0;
uint8_t RPMEXTRA=0;
 //POSIBLE PROBLEMA FUTURO NO 8 BYTES
uint8_t rpm = 0;
uint16_t RPMX_DIV;
uint16_t Freq;
int16_t SPEEDE;
uint32_t PWM_PM;
uint32_t CHECK_PWM_PM;


//unsigned long DatosRX[]
uint8_t muestra;

volatile uint8_t DatosRX[51];
volatile uint8_t indexRX = 0;
uint8_t datosTX[40];
uint8_t indexTX = 0;
//uint16_t overhead;

uint8_t datosx [40];

volatile int i;
volatile bool bandera1= 0;
volatile bool bandera2= 0;
volatile bool bandera3= 0;
volatile bool bandera4= 0;
volatile bool bandera5= 0;
volatile bool bandera6= 0;


volatile uint8_t TiempoRPM = 0;
volatile uint32_t GIROSX;
volatile uint32_t RPMX;
uint32_t  MMS;
volatile uint32_t TIMING;
volatile uint32_t TFULL;

volatile uint16_t RPM;
volatile uint16_t RPMTOTAL;

volatile int8_t revision;
volatile int32_t A=0;

volatile uint32_t GIROS;
volatile uint32_t TSEC_GIROS;

volatile uint32_t WDOG_TIME; //REVISO QUE LOS TIEMPOS SEAN PERFETOS E IMPOLUTOS ...

uint8_t COMUNICACION[30];
uint8_t pos = 0;//IMPORTANTE NO BORRAR PARTE DEL ARREGLO COMUNICACION Y LA FUNCION FRAGMENTAR BYTE


volatile int16_t pulsosR_temp;
volatile int32_t pulsosR;
volatile int32_t pulsosL;

static int32_t lastCountLeft  = 0;
static int32_t lastCountRight = 0;
int32_t dL;
int32_t dR;
uint32_t DistanciaFinal = 200;


int32_t tryL;
int32_t tryR;


float delta_t_s;

#define PI               3.14159265f
#define WHEEL_RADIUS_MM  15.0f
#define PULSES_PER_REV   1135
#define WHEEL_BASE_MM    80.0f


typedef struct{
	uint8_t inicio_EN;
	uint8_t tamano_EN;
	uint8_t *datos_EN;  //RECORDOR NOMBRE
	uint8_t fin_EN;

}ENVIAPAQUETE;

ENVIAPAQUETE np;

typedef struct{
	uint8_t inicio;
	uint8_t tamano;
	uint8_t *datos;  //changeable
	uint8_t crc;
	uint8_t fin;

}PAQUETE;
PAQUETE pk1;


typedef enum {
    ESTADO_AVANCE_TRAPECIO,
    ESTADO_STOP_INTERMEDIO,
    ESTADO_GIRO_TRAPECIO,
    ESTADO_AVANCE_FINAL,
	ESTADO_STOP_INTERMEDIO_2,

} EstadoMovimiento;

EstadoMovimiento estado_mov = ESTADO_AVANCE_TRAPECIO;


typedef enum{  ///// mi timer pro

    TLED=0,  //TIMER EJEMPLO
	TWDOG,
	TTALK,
	TLED_RED,
/*
	TTALK1,  //TIMER ASOCIADO A LA COMUNICACION.
	TTALK2,  //TIMER ASOCIADO A LA COMUNICACION.
	TTALK3,  //TIMER ASOCIADO A LA COMUNICACION.
	TTALK4,  //TIMER ASOCIADO A LA COMUNICACION.
	TTALK5,  //TIMER ASOCIADO A LA COMUNICACION.
	TTALK6,  //TIMER ASOCIADO A LA COMUNICACION.
*/
	TSPEED,  //changeable
	TTOTAL,

}TIMERS;


volatile uint32_t tim_pro[TTOTAL]; //MIS TIMERS CON ISR PRO
uint32_t timers[TTOTAL]  = {0};  // con esto se nombra a todos todos.
uint8_t hola[] = {0xEE,0xF1};



uint32_t adc_0;
uint32_t adc_1;
uint32_t adc_2;
uint32_t adc_3;
uint32_t adc_4;


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



void EnviarPaquete(uint8_t *dat, uint8_t tam){

	pk1.inicio = 0x77; //Start byte
	pk1.tamano = tam + 4;
	pk1.datos=datosx;

    memcpy(&pk1.datos[0], dat, tam + 4 );

	numDatos = serializarPaquete(&pk1, &datosTX);
	pk1.crc=calcularCRC(datosTX, tam+4);
	pk1.fin =0x12;

	CDC_Transmit_FS(datosTX, numDatos);  ///////////////////////////////////////////aQUI

} //way important to transmit


void ReciboPaquete(const PAQUETE* paquete, uint8_t *pos){

	if(!datosTX[0])return -1;
		if(!datosTX[0]==0xff){

		}
}



typedef enum {

    ESTADO_ADV1_START,
    ESTADO_ADV1_WAIT,

    ESTADO_TURN1_START,
    ESTADO_TURN1_WAIT,

    ESTADO_ADV2_START,
    ESTADO_ADV2_WAIT,

    ESTADO_TURN2_START,
    ESTADO_TURN2_WAIT,


	/////////
    ESTADO_ADV3_START,
    ESTADO_ADV3_WAIT,

    ESTADO_TURN3_START,
    ESTADO_TURN3_WAIT,

///////

    ESTADO_STOP,
    ESTADO_LISTO
} EstadoSeq;

static volatile EstadoSeq estado = ESTADO_ADV1_START;



float trapecio_VD_simple(float avance_odo_real, float mm_desired, float dt, float ACELERATION, float VEL_IN)
{
    VEL_FINAL_L         = VEL_IN;          // velocidad máxima deseada
    AceleracionGlobal_L = ACELERATION;     // aceleración constante
    DIST_ACUM_L          = avance_odo_real; // distancia recorrida hasta ahora
    mm_FINAL_L          = mm_desired;      // distancia total a recorrer

    float DIS_ACCE_L = (VEL_FINAL_L * VEL_FINAL_L) / (2.0f * AceleracionGlobal_L);

    float DIS_DESACCE_L = mm_FINAL_L - DIS_ACCE_L;

    if (DIS_DESACCE_L < DIS_ACCE_L) {
        DIS_DESACCE_L = DIS_ACCE_L;
    }

    if (DIST_ACUM_L < DIS_ACCE_L) {

        v_desired_L += AceleracionGlobal_L * dt;
        if (v_desired_L > VEL_FINAL_L) {
            v_desired_L = VEL_FINAL_L;
        }
    }
    else if (DIST_ACUM_L < DIS_DESACCE_L) {

        v_desired_L = VEL_FINAL_L;
    }
    else {

        v_desired_L -= AceleracionGlobal_L * dt;
        if (v_desired_L < 0.0f) {
            v_desired_L = 0.0f;
        }
    }

    return v_desired_L;
}

/*
float trapecio_VD_simple_angulo(float avance_ang_real, float ang_desired, float dt, float accel, float vel_max)
{
    // signo del giro (+1 o -1)
    int signo = (ang_desired >= 0.0f) ? 1 : -1;

    float ang_total_abs = fabsf(ang_desired);
    float ang_acum_abs  = fabsf(avance_ang_real);

    if (ang_acum_abs >= ang_total_abs) {
        return 0.0f;
    }

    float v_acel  = sqrtf(2.0f * accel * ang_acum_abs);
    float v_decel = sqrtf(2.0f * accel * (ang_total_abs - ang_acum_abs));

    float v_abs = v_acel;
    if (v_abs > v_decel)  v_abs = v_decel;
    if (v_abs > vel_max)  v_abs = vel_max;

    float v_min = accel * dt;
    if (v_abs < v_min)    v_abs = v_min;

    return (signo * v_abs)*9;
}  */



float trapecio_VD_simple_angulo(float avance_ang_real,float ang_desired,float dt,float accel,float vel_max,float v_start) {



    int signo = (ang_desired >= 0.0f) ? 1 : -1;

    float D = fabsf(ang_desired);
    float x = fabsf(avance_ang_real);

    v_desired = 0.0f;
    static float last_D     = 0.0f;

    if (D != last_D) {
        v_desired = 0.0f;
        last_D    = D;
    }

    if (x >= D) {
        v_desired = 0.0f;
        return 0.0f;
    }

    float v_theo_acel  = sqrtf(2.0f * accel * x);
    float v_theo_decel = sqrtf(2.0f * accel * (D - x));
    float v_theo       = v_theo_acel;
    if (v_theo > v_theo_decel) v_theo = v_theo_decel;
    if (v_theo > vel_max)      v_theo = vel_max;

    v_desired += accel * dt;

    if (v_desired > v_theo)    v_desired = v_theo;
    if (v_desired < v_start)   v_desired = v_start;

    return (signo * v_desired)*3;
}




int32_t Pwm_By_Speed_normal(float vars) {

		/*       FUNCION QUE DESCRIBE ACELERACION
		 *               y = 0,1641x + 10
		 */
	    int16_t speed = (int16_t) vars;
	    int8_t signo = (speed < 0) ? -1 : 1;
	    int16_t magnitud = (speed < 0) ? -speed : speed;

	    	float y = ((0.1641f * magnitud) + 6.0f);
	    	int32_t PerPWM = (int32_t)roundf(y);
	    	PerPWM *= signo;

	    	if (PerPWM > 100) {
	    	    PerPWM = 100;
	    	}
	    	else if (PerPWM < -100) {
	    	    PerPWM = -100;
	    	}
	    	else if (PerPWM >= 0 && PerPWM <= 4) {
	    	    PerPWM = 0;
	    	}

	    return PerPWM;
	}




uint8_t fragmentacion(uint32_t dato, uint8_t frag_array[], uint8_t posicion) {

	/* MI FUNCION LLENA HACIA LA DERECHA DESDE LA POSICION QUE UNO DIGA LLENA 1 BYTE
	 *  POSICION DEPENDE DE MI Y ARREGLO Y DATO SON LOS QUE UNO QUIERE COMUNICAR PARA LA FUNCION ENVIARPAQUETE();
	 *  ADEMAS ORGANIZA AUTOMATICAMENTE.
	 */
	uint8_t i = 0;
    while (dato > 255) {
        frag_array[posicion + i] = 0xFF;
        dato -= 255;
        i++;
    }

    frag_array[posicion + i] = dato & 0xFF;
    int8_t NFRAG = i+1;
    return NFRAG; //  REVISAR NUMERO DE FRAGMENTACIONES EN VIVO...
}


////////////////////     AQUI NO DEPENDO DE  NADIE   //////////////

uint16_t VSPEED_QT(uint8_t* DatosRX) {

    int16_t magnitud = (int16_t)((DatosRX[4] << 8) | DatosRX[5]);
    int8_t signo = (DatosRX[3] == 0x01) ? -1 : 1;   // 0x01 = negativo  y 0x00 = positivo //AGREGO SIGNO
    return magnitud * signo;
}


int32_t Pwm_By_Speed(void) {

		/*       FUNCION QUE DESCRIBE ACELERACION
		 *               y = 0,1641x + 10
		 */
	    int16_t speed = VSPEED_QT(DatosRX);
	    int8_t signo = (speed < 0) ? -1 : 1;
	    int16_t magnitud = (speed < 0) ? -speed : speed;

	    	float y = ((0.1641f * magnitud) + 7.7f);
	    	int32_t PerPWM = (int32_t)roundf(y);
	    	PerPWM *= signo;

	    	if (PerPWM > 100) {
	    	    PerPWM = 100;
	    	}
	    	else if (PerPWM < -100) {
	    	    PerPWM = -100;
	    	}
	    	else if (PerPWM >= 0 && PerPWM <= 4) {
	    	    PerPWM = 0;
	    	}

	    return PerPWM;
	}



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM10_Init(void);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM11_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  motoresInit(&htim4, TIM_CHANNEL_1, TIM_CHANNEL_2); //MOTORES

  HAL_TIM_Base_Start_IT(&htim11);  //ESTOY TRABAJANDO ISR PARA TIM.  :)
  HAL_TIM_Base_Start(&htim9); //ESTO PUDE CAUSAR DESFASE CREO
  HAL_TIM_Base_Start(&htim1);  // SI SE COMPLICA COLOCAR TIMER5 JAJA 9600/10 = MS
  HAL_TIM_Base_Start(&htim10);

  CONVERTER_INIT(&hadc1, &htim10, canales, puertos_led, pines_led);

  odo_init_reset(&ps);

  delay_us_tim_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  //importante por encoder mode t1, t2.
HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcValue ,5);



   __HAL_TIM_SET_COUNTER(&htim1,0);  ////BORRAR EN UN FUTURO PERO RECTIFICAR QUE NO HACE NADA... INMEDIATAMENTE...

////////////////////BLOQQUE PARA INCICIAR///////////////
    while ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET )
   {

   }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,1);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,0);
    HAL_Delay(2000);

  while (1)
  {


	  ReadAndAverageSensors(avgReadings, 10);
	  TRANS_CONVS = CONV_Y_TRANS();

	      	  pulsosL = (__HAL_TIM_GET_COUNTER(&htim2)); //DEJAR QUIETO
	      	  pulsosR_temp = (__HAL_TIM_GET_COUNTER(&htim3));
	  	  	  pulsosR = (int32_t) pulsosR_temp;  //DEJAR QUIETO*/

	  if (tim_pro[TWDOG] == 0) {

		  dL = pulsosL - lastCountLeft;
		  dR = pulsosR - lastCountRight;
		  lastCountLeft  = pulsosL;
		  lastCountRight = pulsosR;
	      actualizar_odometria(&ps, dL, dR, 0.01f);

	      tim_pro[TWDOG] = 10;
	  }

 /////////////////////ODOMETRIA DEJAR QUIETO, CONFIGURAR DESDE AQUI////////////////////////


	  /*
	   *
	  if (tim_pro[TSPEED] == 0) {

		  trapecio_VD_simple(avance_odo_real, mm_desired, dt, ACELERATION, VEL_IN);
		 velMotores = trapecio_VD_simple((float)ps.avanceLineal, 300, 0.011, 10, 150);

		 trapecio_VD_simple_angulo(avance_ang_real, ang_desired, dt, accel, vel_max);

		  velMotoresR = trapecio_VD_simple_angulo(ps.theta, -3, 0.011, 100, 100);
	  velMotores = trapecio_VD_simple_angulo(ps.theta, 3.5, 0.011, 400, 400, 70);

	  pwmTrapecio = Pwm_By_Speed_normal(velMotores);

	  	  tim_pro[TSPEED] = 11;
	  }

	    motores((pwmTrapecio),(-pwmTrapecio));      */

//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);

	  if (tim_pro[TSPEED] == 0) {
	      tim_pro[TSPEED] = 11;

	      switch (estado_mov) {

	          case ESTADO_AVANCE_TRAPECIO: {
	              float vel = trapecio_VD_simple(ps.avanceLineal, 20000, 0.011, 10, 150);
	              int pwm = Pwm_By_Speed_normal(vel);
	              motores(pwm, pwm);

	              if (TRANS_CONVS.DIST4 < 50 ) {
	                  motores(0, 0);  // STOP intermedio
	                  estado_mov = ESTADO_STOP_INTERMEDIO;
	              }
	              break;
	          }

	          case ESTADO_STOP_INTERMEDIO: {
	              estado_mov = ESTADO_GIRO_TRAPECIO;
	              ps.theta=0;
	              break;
	          }


	          case ESTADO_GIRO_TRAPECIO: {
	              float vel_giro = trapecio_VD_simple_angulo(ps.theta, 3.9, 0.011, 400, 400, 100);
	              int pwm = Pwm_By_Speed_normal(fabsf(vel_giro));
	              motores(pwm * (vel_giro > 0 ? 1 : -1), -pwm * (vel_giro > 0 ? 1 : -1));  //posible error....

	              if (vel_giro == 0.0f) {
	                  estado_mov = ESTADO_AVANCE_FINAL;
	              }
	              break;
	          }

	          case ESTADO_AVANCE_FINAL: {
	              float vel = trapecio_VD_simple(ps.avanceLineal, 20000, 0.011, 100, 180);
	              int pwm = Pwm_By_Speed_normal(vel);
	              motores(pwm, pwm);



	              if (TRANS_CONVS.DIST4 < 100) {

                      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
	                  motores(0, 0);
	                  estado_mov = ESTADO_STOP_INTERMEDIO_2;
	              }

	              break;
	          }

	          case ESTADO_STOP_INTERMEDIO_2: {

	              ps.theta=0;
	              estado_mov = ESTADO_GIRO_TRAPECIO;

	              break;
	          }



	      }
	  }




	  VELOCIDAD_BOT_REAL= velMotores;




      if(tim_pro[TLED]==0){
    	  tim_pro[TLED]=100;     //MIRAR CON OSCILOSCOPIO...
    	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      }


      PUNTOS_X               = ps.x;
      PUNTOS_Y               = ps.y;
      DIS_MOTOR_L            = ps.left_distance_mm;
      DIS_MOTOR_R            = ps.right_distance_mm;
      THETA                  = ps.theta;
      VEL_MOTOR_L            = ps.velMotorL;
      VEL_MOTOR_R            = ps.velMotorR;
      VEL_LINEAL             = ps.velLineal;
      VEL_ANGULAR            = ps.velAngular;
      AVANCE_LINEA           = ps.avanceLineal;   //ESTO SE SUPONE ES LO QUE AVANZO
      LEFT_DISTANCE_MM_ACUM  = ps.left_distance_mm_acum;
      RIGHT_DISTANCE_MM_ACUM = ps.right_distance_mm_acum;


      MMS = VEL_MOTOR_R;

      fragmentacion(MMS, COMUNICACION, 2);
      fragmentacion(VELOCIDAD_BOT_REAL, COMUNICACION, 5);
      fragmentacion(TRANS_CONVS.DIST1, COMUNICACION, 8);
      fragmentacion(TRANS_CONVS.DIST2, COMUNICACION, 9);
      fragmentacion(TRANS_CONVS.DIST3, COMUNICACION, 10);
      fragmentacion(TRANS_CONVS.DIST4, COMUNICACION, 11);



	  if(tim_pro[TTALK]==0){
	   	  	  tim_pro[TTALK]=100;

	   		  //sprintf (texto, "%5u %5u %5u %5u %5u \n", adcValue[0],adcValue[1],adcValue[2],adcValue[3],adcValue[4]);
	   		  //CDC_Transmit_FS(texto, strlen(texto));  //

		 EnviarPaquete(COMUNICACION, 20);

	  }



	  /*  switch(DatosRX[2])
	  {

	  case 0xA1: //PWM_PM = Pwm_By_Speed(); motores(PWM_PM,0);
		  break;

	  case 0x00: motores(0,0); // CHECK_PWM_PM = 0;
		  break;

	  case 0xA2: motores(-80,0);
		  break;

	  }
	    //  __HAL_TIM_SET_COUNTER(&htim1, 0);
      */

	  	  SPEEDE = VSPEED_QT(DatosRX);

/*
	  RPM = ObtenerRPM();
	  Freq = W_frecuencia();
	  MMS =  VELINEAL();


	  revision = fragmentacion(RPM, COMUNICACION, 0);  //FUNCION FRAGMENTACION LLENA A LA DERECHA
	  fragmentacion(MMS, COMUNICACION, 2);
	  fragmentacion(Freq, COMUNICACION, 5);   */
	 // EnviarPaquete(COMUNICACION, 8);


  /////////////IMPORTANTE NO BORRRARRRR/////////////////
	  	  /*
	  if(tim_pro[TTALK]==0){
	   	  	  tim_pro[TTALK]=500;

		//  EnviarPaquete(COMUNICACION, 8);

	  }*/


    	  ////////////////////LAB ANTERIOR/////////////////////

    	//  WDOG_TIME = __HAL_TIM_GET_COUNTER(&htim11)/10; //OBTENGO TIEMPO ESPECIFICO EN MILISEGUNDOS.

    	  /*
    	  sprintf(texto, "%5lu %5lu\n", MMS,TFULL);
    	  CDC_Transmit_FS(texto, strlen(texto));
    	  		  WDOG_TIME=__HAL_TIM_SET_COUNTER(&htim11,0);
    	  */

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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
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

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
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
  htim1.Init.Prescaler = 9600-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 26-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 96-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 96-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 96-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	HAL_ADC_Start_DMA(&hadc1, adcValue,5);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  //  static uint16_t ms_counter = 0; declarado arriba

    if (htim->Instance == TIM11) {

        for (uint16_t i = 0; i < TTOTAL; i++) {
            if (tim_pro[i] != 0) {
                tim_pro[i]--;
            }
        }

    }

}





void CDC_ReceiveCallBack(uint8_t* Buf, uint32_t Len){
	memcpy(DatosRX,Buf,Len);
	indexRX = Len;
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
