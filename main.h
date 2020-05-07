#ifndef MAIN_H
#define MAIN_H

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "sensors/proximity.h"
#include "../lib/e-puck2_main-processor/src/sensors/VL53L0X/VL53L0X.h"
#include <motors.h>

//Project constants for the different parts

//Constants for the image processing mainly
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define PXTOCM					1570.0f

//Constants for the robot management mainly
#define SPEED_DE_CROISIERE		400 			// Speed when in mode normal
#define VITESSE_RECUL			-500			// Speed when obstacle (door) detected and the robot takes distance with obstacle
#define VITESSE_CHARGE			1000			// Speed of the robot when it goes forward to break down an obstacle
#define GOAL_DISTANCE 			40.0f		// Detection distance of obstacles (doors)
#define MAX_DISTANCE 			25.0f
#define DISTANCE_CHARGE			100 			// Set-back distance before breaking down an obstacle (demo 2)
#define VITESSE_APPROCHE_INT		200			// Speed when approaching an intersection
#define VITESSE_ROT_CHEMIN    	250			// Absolute value of speed for the 2 motors when
											// rotating to choose a new path at an intersection

#define PERIMETER_EPUCK			13
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2			// Coefficient used to

//Constants for the PID regulator
#define ERROR_THRESHOLD			5 			// [mm] DELETE le mm ?  because the camera is noisy (we don't want to take noise in consideration
											// during our robot alignment

#define KPL						0.5f  		// Parameters for line alignment PID : values determined according
#define KIL 				   	0.0018f	 	// to automatic lessons and experimental trials
#define KDL 				   	4.0f			//
#define TERM_I_MAX				200			// Maximum integral contribution to PID
#define MAX_SUM_ERROR_L			TERM_I_MAX/KIL //limits sum error

//Constants for RGB leds
#define RGB_INTENSITY 10	

/* Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#endif
