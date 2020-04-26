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
#include "../lib/e-puck2_main-processor/src/epuck1x/utility/utility.h"

static _Bool demo = DEMO1;

#define temp_pause 18300000

//simple PI regulator implementation to manage line alignment and obstacles
int16_t pi_regulator(uint16_t distance, uint16_t goal, _Bool reset){

	int16_t error = 0;
	int16_t speed = 0;

	static int sum_error = 0;
	static int16_t error_pre = 0;

	if(reset)
	{
		sum_error = 0;
		error_pre = 0;
		return 0;
	}

	error = (int)distance - (int)goal;

	//chprintf((BaseSequentialStream *)&SDU1, "error %d", error);

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be aligned and the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;
	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR_L){
		sum_error = MAX_SUM_ERROR_L;
	}else if(sum_error < -MAX_SUM_ERROR_L){
		sum_error = -MAX_SUM_ERROR_L;
	}

	//chprintf((BaseSequentialStream *)&SDU1, "test");
	//chprintf((BaseSequentialStream *)&SDU1, "sum error %d", sum_error);

	if(error_pre)
		speed = KPL * error + KIL * sum_error + KDL*(error-error_pre);
	else
		speed = KPL * error + KIL * sum_error;

	error_pre = error;
    return speed;
}

_Bool choix_chemin(int16_t* vitesse_rotation)
{

	static int8_t recherche_chemin = RIGHT;
	static uint8_t compteur = 0;

	switch(recherche_chemin)
	{
		//Dans un premier temps vérifie s'il ya un chemin à droite de l'intersection, si oui y va
		//300 correspond à (PERIMETER_EPUCK*CONV_CM2STEP/4+1) et considérations empiriques pour bien touner à 90°
		case RIGHT:
			*vitesse_rotation = VITESSE_ROT_CHEMIN;
			set_led(LED3,1);
			if((abs(left_motor_get_pos()) >= 300) && (abs(right_motor_get_pos()) >= 300)) //MAGIC NB
			{
				//right_motor_set_speed(0);
				//left_motor_set_speed(0);

				//wait(temp_pause); //semaphore capture image aller et revenir
				//chThdSleep /Exit/Resume

				//ajouter un compteur
				*vitesse_rotation = 0;

				if(compteur++ >= 20) // attente d'avoir une nouvelle image et un process
				{
					compteur = 0;
					if(get_std_dev() > 18)  //utiliser line_not_found peut-être si cette contiion marche pas bien  //MAGIC NB
					{		//abs(get_line_position()-IMAGE_BUFFER_SIZE/2)<100
						return TRUE; //chemin sélectionné
					}
					else
					{
						recherche_chemin = FRONT;
						set_led(LED3,0);
						left_motor_set_pos(0);
						right_motor_set_pos(0);
					}
				}
			}
			return FALSE;
			break;

		case FRONT:
			set_rgb_led(LED2,200,0,0);
			set_rgb_led(LED8,200,0,0);
			*vitesse_rotation = -VITESSE_ROT_CHEMIN;
			if((abs(left_motor_get_pos()) >= 300) && (abs(right_motor_get_pos()) >= 300)) //MAGIC NB
			{
				*vitesse_rotation = 0;

				if(compteur++ >= 20) // attente d'avoir une nouvelle image et un process
				{
					compteur = 0;
					set_rgb_led(LED2,0,0,0);
					set_rgb_led(LED8,0,0,0);
					if(get_std_dev() > 18)  //utiliser line_not_found peut-être si cette contiion marche pas bien  //MAGIC NB
					{
						recherche_chemin = RIGHT;
						return TRUE; //chemin sélectionné
					}
					else
						recherche_chemin = LEFT;

				}
			}
			return FALSE;
			break;

		case LEFT:
			set_led(LED7,1);
			*vitesse_rotation = -VITESSE_ROT_CHEMIN;
			if((abs(left_motor_get_pos()) >= 600) && (abs(right_motor_get_pos()) >= 600)) //MAGIC NB
			{
				*vitesse_rotation = 0;

				if(compteur++ >= 20) // attente d'avoir une nouvelle image et un process
				{
					compteur = 0;
					if(get_std_dev() > 18)  //utiliser line_not_found peut-être si cette contiion marche pas bien  //MAGIC NB
					{
						recherche_chemin = RIGHT; //reinitialise pour prochain
						set_led(LED7,0);
						return TRUE; //chemin sélectionné
					}
					else
						recherche_chemin = BACK;
				}
			}
			return FALSE;
			break;

		case BACK:
			set_led(LED5,1);
			*vitesse_rotation = -VITESSE_ROT_CHEMIN;
			if((abs(left_motor_get_pos()) >= 900) && (abs(right_motor_get_pos()) >= 900)) //MAGIC NB
			{
				set_led(LED5,0);

				*vitesse_rotation = 0;

				if(compteur++ >= 20) // attente d'avoir une nouvelle image et un process
				{
					compteur = 0;

				//on suppose que dans tous les cas il y a une ligne
				//if(get_std_dev() > 5)  //utiliser line_not_found peut-être si cette contiion marche pas bien  //MAGIC NB
				//{
					recherche_chemin = RIGHT;
					return TRUE; //chemin sélectionné
				//}
				//else
					//recherche_chemin = LEFT;
				}
			}
			return FALSE;
			break;

		default:
			chprintf((BaseSequentialStream *)&SDU1, "MODE ERROR");
			return FALSE;
	}
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
    static _Bool condition_degommage = true;

    while(1){
        time = chVTGetSystemTime();
        
        //distance_mm gives the distance of the detected obstacle from the robot thanks to the TOF
        uint16_t distance_mm;
        distance_mm = VL53L0X_get_dist_mm();

        //IF WE ARE NOT ABLE TO IMPLEMENT A SATISFYING LINE ALIGNMENT PID
        //computes a correction factor to let the robot rotate to be in front of the line
        //speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));

        _Bool intersection = (get_std_dev() <= 10.5) ;

        //mode change
        switch(mode)
        {
        		case NORMAL:
        			//statements
        			speed = SPEED_DE_CROISIERE;
        			//set_front_led(1);
        			set_led(LED1,1);

        			//chprintf((BaseSequentialStream *)&SDU1, "std_dev %f", get_std_dev());
        			//chprintf((BaseSequentialStream *)&SDU1, "line width %d", get_line_width());
        			//chprintf((BaseSequentialStream *)&SDU1, "line pos %d \n", get_line_position());
        			speed_correction = pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, 0);

        			        //if the line is nearly in front of the camera, don't rotate
        			if(abs(speed_correction) < ROTATION_THRESHOLD || speed == 0){
        	         	speed_correction = 0;
        			}

        			//si distance est initialise (different de 0)
        			if(distance_mm && (distance_mm < GOAL_DISTANCE)) //si on est dans demo 2 et obstacle détecté on rentre en mode obstacle
        			{
        				if(demo) mode = OBSTACLE;
        				else // clean avec un mode demi-tour??
        				{

        					set_rgb_led(LED4,200,0,0);
        					set_rgb_led(LED6,200,0,0);
        					set_led(LED1,1);
        					left_motor_set_pos(0);
        					right_motor_set_pos(0);
        					right_motor_set_speed(-ROTATION_COEFF*VITESSE_ROT_CHEMIN);
        					left_motor_set_speed(ROTATION_COEFF*VITESSE_ROT_CHEMIN);
        					while((abs(left_motor_get_pos()) <= 655) && (abs(right_motor_get_pos()) <= 655));
        					speed_correction = 0;
        					set_rgb_led(LED4,0,0,0);
        					set_rgb_led(LED6,0,0,0);
        					set_led(LED1,0);
						// mode reste normal, demi tour devant l'obstacle effectué (démo 1)
        					//reset sum_error
        				}
        			}
        			else if(intersection)
        			{
        				mode = INTERSECTION;
        				//set_front_led(0);
        				left_motor_set_pos(0);
        				right_motor_set_pos(0);
        			}
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
        			speed_correction = 0; // pas sure de ça, pas ouf si le robot s'est d�cal�, essai avec une correcton?
        			set_led(LED5,0);
        			speed = MOTOR_SPEED_LIMIT;
        			if(distance_mm >= 110)
        			{
        				pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, 1); //reset
        				mode = NORMAL;
        			}

        			break;

        		case INTERSECTION:
        			//statements
        			speed_correction = 0;
        			speed = VITESSE_APPROCHE_INT;
        			set_led(LED1,0);
        			set_rgb_led(LED2,200,200,0);
        			set_rgb_led(LED4,200,200,0);
        			set_rgb_led(LED6,200,200,0);
        			set_rgb_led(LED8,200,200,0);
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
        			speed = 0;
        			set_rgb_led(LED2,0,0,0);
        			set_rgb_led(LED4,0,0,0);
        			set_rgb_led(LED6,0,0,0);
        			set_rgb_led(LED8,0,0,0);
        			if(choix_chemin(&speed_correction))
        			{
        				mode = NORMAL;
        				speed_correction = pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, 1); //reset
        			}
        			break;

        		default:
        			chprintf((BaseSequentialStream *)&SDU1, "MODE ERROR");
        	}

        //applies the speed from the PI regulator and the correction for the rotation

		right_motor_set_speed(speed - ROTATION_COEFF*speed_correction); //ROTATION_COEFF * à enlever si PID
		left_motor_set_speed(speed + ROTATION_COEFF*speed_correction); //ROTATION_COEFF * à enlever si PID

        //10Hz soit 100ms d'attente
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void rob_management_start(void){
	chThdCreateStatic(waRob_management, sizeof(waRob_management), NORMALPRIO+10, Rob_management, NULL);
}
