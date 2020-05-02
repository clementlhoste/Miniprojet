#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "sensors/proximity.h"
#include "../lib/e-puck2_main-processor/src/sensors/VL53L0X/VL53L0X.h"
#include <motors.h>
#include "msgbus/messagebus.h"

//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define PXTOCM					1570.0f // Experimental value used for the conversion of pixel data to cm

#define SPEED_DE_CROISIERE		400 		// vitesse de déplacement lorsque mode normal
#define VITESSE_RECUL			-500		// vitesse de déplacement lorsque obstacle détecté
#define VITESSE_CHARGE			1000		//vitesse de d�placement apr�s avoir recul�
#define GOAL_DISTANCE 			40.0f	// Distance de détection de l'obstacle
#define MAX_DISTANCE 			25.0f
#define DISTANCE_CHARGE			100 		// distance de recul du robot avant de dégommer l'obstacle (démo 2)
#define VITESSE_APPROCHE_INT	200
#define VITESSE_ROT_CHEMIN    	250
#define PERIMETER_EPUCK			13
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2

#define ERROR_THRESHOLD			5 	// [mm] because of the noise of the TOF
#define KPO						200.0f	// Parameters for the obstacle management PID
#define KIO 					1.6f  	// must not be zero
#define KPL						0.5f  	// Parameters for line alignment PID --> A REESSAYER
#define KIL 				    0.0018f//0.002f0.015f
#define KDL 				    4.0f//2.0f
//0.00075f
//0.00043
#define MAX_SUM_ERROR_O			(MOTOR_SPEED_LIMIT/KIO)
#define MAX_SUM_ERROR_L			100/KIL
//((MOTOR_SPEED_LIMIT-SPEED_DE_CROISIERE)/KIL)


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
