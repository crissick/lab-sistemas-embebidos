/* File: mms_odometria.c */
#include "mms_odometria.h"

int32_t difL;
int32_t difR;

// Inicializa pose y contadores
void odo_init_reset(Pose *pose)
{
    // Reset de pose
    pose->x = 0.0f;
    pose->y = 0.0f;
    pose->theta = 0.0f;
    pose->velMotorL = 0.0f;
    pose->velMotorR = 0.0f;

    pose->velLineal = 0.0f;

    pose->velAngular = 0.0f;
    pose->avanceLineal = 0.0f;

    pose->left_distance_mm = 0.0f;
    pose->right_distance_mm = 0.0f;

    pose->left_distance_mm_acum = 0.0f;
    pose->right_distance_mm_acum = 0.0f;

}

// Calcula odometría: diferencias, integración midpoint, velocidades
void actualizar_odometria(Pose *pose, int32_t countL, int32_t countR, float dt)
{

	float difL = (float)countL;
	    float difR = (float)countR;

	    float dmmLeft  = (difL * 2.0f * PI * WHEEL_RADIUS_MM) / PULSES_PER_REV;
	    float dmmRight = (difR * 2.0f * PI * WHEEL_RADIUS_MM) / PULSES_PER_REV;

	    // Acumular distancias
	    pose->left_distance_mm_acum  += dmmLeft;
	    pose->right_distance_mm_acum += dmmRight;
	    pose->left_distance_mm  = dmmLeft;
	    pose->right_distance_mm = dmmRight;

	    // Incrementos lineal y angular
	    float delta_lineal  = 0.5f * (dmmLeft + dmmRight);
	    float delta_angular = (dmmRight - dmmLeft) / WHEEL_BASE_MM;

	    // Integración usando ángulo medio
	    float theta_prev = pose->theta;
	    float theta_mid  = theta_prev + 0.5f * delta_angular;
	    float theta_new  = theta_prev + delta_angular;

	    // Wrap manual rápido a [-2π, +2π] usando solo comparaciones y restas
	    const float limit = 2.0f * PI;
	    if (theta_new >  limit) theta_new -= limit;
	    else if (theta_new < -limit) theta_new += limit;

	    pose->theta = theta_new;
	    pose->x    += delta_lineal * cosf(theta_mid);
	    pose->y    += delta_lineal * sinf(theta_mid);

	    // Velocidades y avance acumulado
	    pose->velLineal    = delta_lineal  / dt;
	    pose->velAngular   = delta_angular / dt;
	    pose->velMotorL    = dmmLeft  / dt;
	    pose->velMotorR    = dmmRight / dt;
	    pose->avanceLineal += delta_lineal;
}
