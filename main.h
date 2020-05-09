#ifndef MAIN_H
#define MAIN_H

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "sensors/proximity.h"
#include "../lib/e-puck2_main-processor/src/sensors/VL53L0X/VL53L0X.h"
#include <motors.h>

//Constant used in different modules
#define PERIMETER_EPUCK			13                   // [cm]
#define CONV_CM2STEP			1000/PERIMETER_EPUCK // [step/cm] 
#define GOAL_DISTANCE 			40					 // [mm] Detection distance of obstacles (doors)

/* Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#endif
