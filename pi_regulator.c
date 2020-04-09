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
#include "../lib/e-puck2_main-processor/src/leds.h"

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
    //mode helps to make decisions
    static int8_t mode = NORMAL;


    while(1){
        time = chVTGetSystemTime();
        
        //distance_cm is modified by the image processing thread
        uint16_t distance_mm;
        distance_mm = VL53L0X_get_dist_mm();

        chprintf((BaseSequentialStream *)&SDU1, "Distance= %f\n", distance_mm);

        //test � clean:
        //chprintf((BaseSequentialStream *)&SDU1, "line width= %d\n", line_width);
        //chprintf((BaseSequentialStream *)&SDU1, "line pos= %d\n", get_line_position());
        _Bool intersection = (line_width && line_width < 120); //line_width > 290 ||

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
        		//chprintf((BaseSequentialStream *)&SDU1, "mode norm\n");
        		set_led(LED1,1);

        	break;

        	case OBSTACLE:
        		//statements
        		set_led(LED5,1);
        		speed_correction= 0; //bloquer
    			speed = -500; //MAGIC NB
        		if(distance_mm >= 1000) //MAGIC NUMBER
        			mode = ATTAQUE;
        	break;

        	case ATTAQUE:
        		//statements
        		speed_correction = 0;
        		set_led(LED5,0);
        		speed = 700;
        		if(distance_mm >= 110)
        			mode = NORMAL;
        	break;

        	case INTERSECTION:
        		//statements
        		speed_correction = 0;
        		speed = 300;
        		//chprintf((BaseSequentialStream *)&SDU1, "line width= %d\n", line_width);
        		set_led(LED1,0);
        		set_body_led(1);
        		if(left_motor_get_pos()>= 270 && right_motor_get_pos()>= 270) // 1.5*CONV_CM2STEP MAGIC NB
        		{
        		    speed=0;
        		    left_motor_set_pos(0);
        		    right_motor_set_pos(0);
        		    //chprintf((BaseSequentialStream *)&SDU1, "Pause\n");
        			mode = CHOIX_CHEMIN;
        			//chprintf((BaseSequentialStream *)&SDU1, "va tourner\n");
        		}

        		//if(abs(left_motor_get_pos())>=PERIMETER_EPUCK/4 && abs(right_motor_get_pos())>=PERIMETER_EPUCK/4)
        			//mode = NORMAL;
        	break;

        	case CHOIX_CHEMIN:
        	//statements
        		speed_correction = 280;
        		speed = 0;
        		set_led(LED3,1);
        		if((abs(left_motor_get_pos())>= 270) && (abs(right_motor_get_pos())>= 270)) //(PERIMETER_EPUCK*CONV_CM2STEP/4+1)
        		{
        			speed=0;
        			//chprintf((BaseSequentialStream *)&SDU1, "Pause2\n");
        			//chThdSleepMilliseconds(1000);
        	        mode = NORMAL;
        	        set_led(LED3,0);
        	        set_body_led(0);
        	        //chprintf((BaseSequentialStream *)&SDU1, "Retour norm\n");
        		}
        	    //chprintf((BaseSequentialStream *)&SDU1, "mode choix chemin\n");
        	break;

        	default:
        		chprintf((BaseSequentialStream *)&SDU1, "MODE ERORR");
        }

       // chprintf((BaseSequentialStream *)&SDU1, "speed, speed cor %d, %d\n", speed, speed_correction);

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
