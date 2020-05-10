#ifndef MAIN_H
#define MAIN_H

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "sensors/proximity.h"
#include "../lib/e-puck2_main-processor/src/sensors/VL53L0X/VL53L0X.h"
#include <motors.h>


/* Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#endif
