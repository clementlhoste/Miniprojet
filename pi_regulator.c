#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include "..\lib\e-puck2_main-processor\src\sensors\VL53L0X\VL53L0X.h"

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
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

    return (int16_t)speed;
}


static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;
    //mode helps to make decisions
    static int8_t mode = NORMAL;


    while(1){
        time = chVTGetSystemTime();
        
        //distance_cm is modified by the image processing thread
        float distance_mm;
        distance_mm = VL53L0X_get_dist_mm();

        uint16_t line_width = get_line_width();

        //test à clean:
        //chprintf((BaseSequentialStream *)&SDU1, "line width= %d\n", line_width);
        //chprintf((BaseSequentialStream *)&SDU1, "line pos= %d\n", get_line_position());
        _Bool intersection = line_width > 290 || line_width < 100;

        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD || speed == 0){
        	speed_correction = 0;
        }

        //mode change
        switch(mode)
        {
        	case NORMAL:
        		//statements
        		speed = 500; //MAGIC NB
        		if(distance_mm <= GOAL_DISTANCE)
        			mode = OBSTACLE;
        		else if(intersection)
        		{
        			mode = INTERSECTION;
        			left_motor_set_pos(0);
        			right_motor_set_pos(0);
        		}

        	break;

        	case OBSTACLE:
        		//statements
        		speed_correction= 0; //bloquer
    			speed = -500; //MAGIC NB
        		if(distance_mm >= 100) //MAGIC NUMBER
        			mode = ATTAQUE;
        	break;

        	case ATTAQUE:
        		//statements
        		speed_correction = 0;
        		speed = 700;
        		if(distance_mm >= 110)
        			mode = NORMAL;
        	break;

        	case INTERSECTION:
        		//statements
        		speed_correction = 300;
        		speed = 0;
                chprintf((BaseSequentialStream *)&SDU1, "mode int= %d\n", left_motor_get_pos());
        		if(abs(left_motor_get_pos())>=PERIMETER_EPUCK/4 && abs(right_motor_get_pos())>=PERIMETER_EPUCK/4)
        			mode = NORMAL;
        	break;

        	default:
        		chprintf((BaseSequentialStream *)&SDU1, "MODE ERORR");
        }

       // chprintf((BaseSequentialStream *)&SDU1, "speed, speed cor %d, %d\n", speed, speed_correction);

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
		left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO+10, PiRegulator, NULL);
}
