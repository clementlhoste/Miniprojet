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

//Project constants for the different parts

//constants for the image processing mainly
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define PXTOCM					1570.0f

//constants for the robot management mainly
#define SPEED_DE_CROISIERE		400 			// vitesse de déplacement lorsque mode normal
#define VITESSE_RECUL			-500			// vitesse de déplacement lorsque obstacle détecté
#define VITESSE_CHARGE			1000			// vitesse de déplacement après avoir reculé
#define GOAL_DISTANCE 			40.0f		// Detection istance de détection de l'obstacle
#define MAX_DISTANCE 			25.0f
#define DISTANCE_CHARGE			100 			// Setback distance before breaking down an obstacle (demo 2)
#define VITESSE_APPROCHE_INT	200
#define VITESSE_ROT_CHEMIN    	250
#define PERIMETER_EPUCK			13
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2

//constants for the PID regulator
#define ERROR_THRESHOLD			5 			// [mm] because of the noise of the camera
#define KPL						0.5f  		// Parameters for line alignment PID
#define KIL 				   		0.0018f	 	// Values determined according to automatic lessons and
#define KDL 				   		4.0f
#define TERM_I_MAX				200			//maximum integral contribution to PID
#define MAX_SUM_ERROR_L			TERM_I_MAX/KIL //limits sum error

#define RGB_INTENSITY 10	


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#endif
