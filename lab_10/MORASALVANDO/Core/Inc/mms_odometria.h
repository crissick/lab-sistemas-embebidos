/* File: mms_odometria.h */
#ifndef INC_MMS_ODOMETRIA_H_
#define INC_MMS_ODOMETRIA_H_

#include <stdint.h>
#include <math.h>

// --- constantes ---
#define PI               3.14159265f
#define WHEEL_RADIUS_MM  15.0f    // radio de rueda en mm
#define PULSES_PER_REV   1135     // pulsos mecánicos por revolución
#define WHEEL_BASE_MM    80.0f    // distancia entre ruedas en mm

// Estructura Pose: posición, orientación, velocidades y recorridos
typedef struct {
    float x;
    float y;
    float theta;
    float velMotorL;
    float velMotorR;
    float velLineal;
    float velAngular;
    float avanceLineal;
    float left_distance_mm;
    float right_distance_mm;
    float left_distance_mm_acum;
    float right_distance_mm_acum;
} Pose;

void odo_init_reset(Pose *pose);

// Actualiza odometría. Llamar cada dt segundos con counts en pulsos mecánicos.
// countL, countR: contadores de encoder (pulsos mecánicos)
// dt: intervalo en segundos (ej. 0.01f = 10 ms)
void actualizar_odometria(Pose *pose, int32_t countL, int32_t countR, float dt);

#endif /* INC_MMS_ODOMETRIA_H_ */
