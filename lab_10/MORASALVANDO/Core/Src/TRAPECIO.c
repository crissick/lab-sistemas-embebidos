/*
 * TRAPECIO.c
 *
 *  Created on: May 10, 2025
 *      Author: YEISON LARA
 */

#include "TRAPECIO.h"


/*
float TIEMPO_ACUM = 0;
float DIST_ACUM = 0;
float VEL_FINAL = 0;
float mm_FINAL = 0;
float DIS_ACCE = 0;
float DIS_DESACCE = 0;
float AceleracionGlobal = 0;
static float v_desired = 0.0f;
float VEL_INICIAL_GLOBAL = 0.0f;

float trapecio(float avance_odo_real, float mm_desired, float dt, float ACELERATION, float VEL_IN, float VEL_INICIAL)
{

    //static float v_desired   = 0.0f;

     VEL_FINAL         = VEL_IN;          // velocidad máxima deseada
     AceleracionGlobal = ACELERATION;     // aceleración constante
     DIST_ACUM          = avance_odo_real; // distancia recorrida hasta ahora
     mm_FINAL          = mm_desired;      // distancia total a recorrer
     VEL_INICIAL_GLOBAL = VEL_INICIAL;

     DIS_ACCE = (VEL_FINAL * VEL_FINAL) / (2.0f * AceleracionGlobal);

     //DIS_ACCE = (v_desired * v_desired) / (2.0f * AceleracionGlobal);

     // Punto donde empieza la desaceleración
     DIS_DESACCE = mm_FINAL - DIS_ACCE;
     if (DIS_DESACCE < DIS_ACCE) {
    	 DIS_DESACCE = DIS_ACCE;
     }

     if (DIST_ACUM < DIS_ACCE)
     {
    	 v_desired += (AceleracionGlobal * dt) + VEL_INICIAL_GLOBAL;
    	 if (v_desired > VEL_FINAL) {
            v_desired = VEL_FINAL;
        }
    }
    else if (DIST_ACUM < DIS_DESACCE) {


        v_desired = VEL_FINAL;
    }
    else {

        v_desired -= AceleracionGlobal * dt;
        if (v_desired < 0.0f) {
            v_desired = 0.0f;
        }
    }

    return v_desired;
}*/

