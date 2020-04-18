#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <robot_management.h>
#include "../lib/e-puck2_main-processor/src/sensors/VL53L0X/VL53L0X.h"
#include "../lib/e-puck2_main-processor/src/leds.h"

//simple PI regulator implementation to manage line alignment and obstacles
int16_t pi_regulator(uint16_t distance, uint16_t goal, int8_t mode){

	int16_t error = 0;
	int16_t speed = 0;

	static int sum_error = 0;

	error = (int)distance - (int)goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the ToF is a bit noisy.
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	if(mode == NORMAL) //Line alignment PID
	{
		//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
		if(sum_error > MAX_SUM_ERROR_L/2){
			sum_error = MAX_SUM_ERROR_L;
		}else if(sum_error < -MAX_SUM_ERROR_L/2){
			sum_error = -MAX_SUM_ERROR_L;
		}

		speed = KPL * error + KIL * sum_error;
	}

	else if(mode == OBSTACLE) //Obstacle management PID
	{
		//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
		if(sum_error > MAX_SUM_ERROR_O/2){
			sum_error = MAX_SUM_ERROR_O;
		}else if(sum_error < -MAX_SUM_ERROR_O/2){
			sum_error = -MAX_SUM_ERROR_O;
		}

		speed = KPO * error + KIO * sum_error;
	}

    return speed;
}


static THD_WORKING_AREA(waRob_management, 256);
static THD_FUNCTION(Rob_management, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //a commenter plus tard
    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;

    //mode variable is a state variable used to adapt the epuck2 behavior
    //demo varaiable is a state variable used to determine which part of the demonstration we are proceeding
    //DEMO1 : line alignment, robot moving through the maze, displacement algorithm, management of obstacles as walls
    //DEMO2 : same but now the robot can go through obstacles like a battering ram, voice recognition, voice command "go" etc..PNN
    static int8_t mode = NORMAL;
    static _Bool demo = DEMO1;
    static _Bool condition_degommage = true;

    while(1){
        time = chVTGetSystemTime();
        
        //distance_mm gives the distance of the detected obstacle from the robot thanks to the TOF
        uint16_t distance_mm;
        distance_mm = VL53L0X_get_dist_mm();

        //IF WE ARE NOT ABLE TO IMPLEMENT A SATISFYING LINE ALIGNMENT PID
        //computes a correction factor to let the robot rotate to be in front of the line
        speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
        //speed_correction = pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, mode);

        //if the line is nearly in front of the camera, don't rotate
        if(abs(speed_correction) < ROTATION_THRESHOLD || speed == 0){
        	speed_correction = 0;
        }

        _Bool intersection = (get_std_dev() <= 2) ;

        //mode change
        switch(mode)
        {
        		case NORMAL:
        			//statements
        			speed = SPEED_DE_CROISIERE;
        			if(distance_mm <= GOAL_DISTANCE)
        				mode = OBSTACLE;
        			else if(intersection)
        			{
        				mode = INTERSECTION;
        				left_motor_set_pos(0);
        				right_motor_set_pos(0);
        			}
        			set_led(LED1,1);
        			break;

        		case OBSTACLE:
        			//statements
        			set_led(LED5,1);
        			speed_correction = 0; // bloquer la rotation
        			speed = VITESSE_RECUL;
        			if(distance_mm >= DISTANCE_CHARGE)
        			{
        				speed = 0;
        				if(condition_degommage) mode = ATTAQUE;
        			}

        			break;

        		case ATTAQUE:
        			//statements
        			speed_correction = 0; // pas sure de ca
        			set_led(LED5,0);
        			speed = MOTOR_SPEED_LIMIT;
        			if(distance_mm >= 110)
        				mode = NORMAL;
        			break;

        		case INTERSECTION:
        			//statements
        			speed_correction = 0;
        			speed = VITESSE_APPROCHE_INT;
        			set_led(LED1,0);
        			set_rgb_led(LED2,200,0,0);
        			if(left_motor_get_pos()>= 350 && right_motor_get_pos()>= 350) // 1.5*CONV_CM2STEP MAGIC NB
        			{
        				speed=0;
        				left_motor_set_pos(0);
        				right_motor_set_pos(0);
        				mode = CHOIX_CHEMIN;
        			}
        			break;

        		case CHOIX_CHEMIN:
        			//statements
        			speed_correction = 250;
        			speed = 0;
        			set_led(LED3,1);
        			if((abs(left_motor_get_pos())>= 300) && (abs(right_motor_get_pos())>= 300)) //(PERIMETER_EPUCK*CONV_CM2STEP/4+1)
        			{
        				speed=0;
        				mode = NORMAL;
        				set_led(LED3,0);
        				set_rgb_led(LED2,0,0,0);
        			}
        			break;

        		default:
        			chprintf((BaseSequentialStream *)&SDU1, "MODE ERROR");
        	}

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(speed - speed_correction); //ROTATION_COEFF *
		left_motor_set_speed(speed + speed_correction); //ROTATION_COEFF *

        //10Hz soit 100ms d'attente
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void rob_management_start(void){
	chThdCreateStatic(waRob_management, sizeof(waRob_management), NORMALPRIO+10, Rob_management, NULL);
}
