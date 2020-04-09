#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include "../lib/e-puck2_main-processor/src/sensors/VL53L0X/VL53L0X.h"

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	uint16_t error = 0;
	uint16_t speed = 0;

	static int sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the ToF is a bit noisy.
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    while(1){
        time = chVTGetSystemTime();
        
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
        uint16_t distance_mm;
        distance_mm = VL53L0X_get_dist_mm();

        chprintf((BaseSequentialStream *)&SDU1, "Distance= %d\n", distance_mm);


        speed = pi_regulator(distance_mm, GOAL_DISTANCE);
        //speed = 300;

        //computes a correction factor to let the robot rotate to be in front of the line
        //speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

        //if the line is nearly in front of the camera, don't rotate
       // if(abs(speed_correction) < ROTATION_THRESHOLD){
        	speed_correction = 0;
       // }

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

        //10Hz soit 100ms d'attente
        chThdSleepUntilWindowed(time, time + MS2ST(100));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO+10, PiRegulator, NULL);
}
