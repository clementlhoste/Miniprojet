#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define ROTATION_COEFF			2
#define PXTOCM					1570.0f // Experimental value used for the conversion of pixel data to cm
#define GOAL_DISTANCE 			40.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			5.0f 	// [mm] because of the noise of the TOF
#define SPEED_DE_CROISIERE		400 		// vitesse de déplacement lorsque mode normal
#define VITESSE_RECUL			-500		// vitesse de déplacement lorsque obstacle détecté
#define KPO						200.0f	// Parameters for the obstacle management PID
#define KIO 					    1.6f  	// must not be zero

#define KPL						2.0f  	// Parameters for line alignment PID --> A REESSAYER
#define KIL 				     	0.0f

#define MAX_SUM_ERROR_O			(MOTOR_SPEED_LIMIT/KIO)
#define MAX_SUM_ERROR_L			(700/KIL) //MAGIC NB
#define PERIMETER_EPUCK			13

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
