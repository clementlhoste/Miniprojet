#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <main.h>
#include <motors.h>
#include <process_image.h>
#include <audio_processing.h>
#include <robot_management.h>
#include "../lib/e-puck2_main-processor/src/leds.h"
#include "../lib/e-puck2_main-processor/src/selector.h"

//PID regulator implementation to manage line alignment and obstacles
int16_t pi_regulator(uint16_t distance, uint16_t goal, _Bool reset){

	int16_t error = 0;
	int16_t speed = 0;

	static int sum_error = 0;
	static int16_t error_pre = 0;

	//if the robot is changing his direction, reset the integral term
	if(reset)
	{
		sum_error = 0;
		error_pre = 0;
		return 0;
	}

	error = (int)distance - (int)goal;


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

	//activate the Kd coefficient only if error_pre exist
	if(error_pre)
		speed = KPL * error + KIL * sum_error + KDL*(error-error_pre);
	else
		speed = KPL * error + KIL * sum_error;

	error_pre = error;
	
    return speed;
}


//state 1 = switch on led, state 0 = switch off 
void gerer_led(int8_t mode, unsigned int state) //uin8_t ?
{
	switch(mode)
	{
	  	case NORMAL:
        //switch the front LED
        	set_led(LED1, state);
        	break;		

	    case DEMI_TOUR:
	    //switch 2 back RGB LED (in RED here)
			set_rgb_led(LED4,(state * 200),0,0); 
			set_rgb_led(LED6,(state *200),0,0);
			break;

	    case OBSTACLE:
	    //switch back LED
	     	set_led(LED5, state);
	       	break;

	    case ATTAQUE:
	    //switch 4 RGB LED in orange
	    //rgb(255,165,0)
	      	set_rgb_led(LED2,(state*255),(state*165),0);
	        set_rgb_led(LED4,(state*255),(state*165),0);
        	set_rgb_led(LED6,(state*255),(state*165),0);
	        set_rgb_led(LED8,(state*255),(state*165),0);
	       	break;

        case INTERSECTION:
	    //switch 4 RGB LED in blue
	    //rgb(30,144,255), magic numbers
	        set_rgb_led(LED2,(state*30),(state*144),(state*255));
	        set_rgb_led(LED4,(state*30),(state*144),(state*255));
        	set_rgb_led(LED6,(state*30),(state*144),(state*255));
	        set_rgb_led(LED8,(state*30),(state*144),(state*255));
	        break;

	    case CHOIX_CHEMIN:
	    //only possible to switch off this light, on is in gerer_led_inter
	    	set_led(LED3, 0);
	   		break;

	    case END:
	    //nothing
	   		break;

	    default:
	 		chprintf((BaseSequentialStream *)&SDU1, "MODE ERROR gestion led");
	}
}

//comments
void mode_led(int8_t mode)
{
	static int8_t ancien_mode = NORMAL;

	if(mode != ancien_mode)
	{
		//�teindre les anciennes LEDS avec ancien_mode
		gerer_led(ancien_mode, 0);
		
		if(mode != CHOIX_CHEMIN) //cas dans gerer_led_inter()
		{	
			//allumer les nouvelles avec mode
			gerer_led(mode, 1);
		}
		ancien_mode = mode;
	}
}

void gerer_led_inter(int8_t dir, unsigned int state)
{
	switch(dir)
	{
	  	case RIGHT:
        //switch the front LED
        	set_led(LED3, state);
        	break;		

	    case FRONT:
	    //switch 2 front RGB LED (in RED here)
			set_rgb_led(LED2,(state*200),0,0);
			set_rgb_led(LED8,(state*200),0,0);
			break;

	    case LEFT:
	    //switch left LED
	     	set_led(LED7,state);
	       	break;

	    case BACK:
	    //switch back LED
	   		set_led(LED5,state);
	       	break;

	    default:
	 		chprintf((BaseSequentialStream *)&SDU1, "MODE ERROR gest led inter");
	}
}


void mode_inter_led(int8_t dir)
{
	static uint8_t ancienne_dir = RIGHT;

	if(dir != ancienne_dir)
	{
		//�teindre les anciennes LEDS avec ancienne_dir
		gerer_led_inter(ancienne_dir, 0);
		//allumer les nouvelles avec mode
		gerer_led_inter(dir, 1);
	}
	ancienne_dir = dir;
}

_Bool choix_chemin(int16_t* vitesse_rotation)
{

	static int8_t recherche_chemin = RIGHT;
	_Bool chemin_trouve = FALSE;
	static uint8_t compteur = 0;

	switch(recherche_chemin)
	{
		case RIGHT: //Dans un premier temps vérifie s'il ya un chemin à droite de l'intersection, si oui y va

			*vitesse_rotation = VITESSE_ROT_CHEMIN;

			//300 correspond à (PERIMETER_EPUCK*CONV_CM2STEP/4+1) et considérations empiriques pour bien touner à 90°
			if((abs(left_motor_get_pos()) >= 315) && (abs(right_motor_get_pos()) >= 315)) //MAGIC NB
			{
				*vitesse_rotation = 0;
				if(compteur++ >= 20) // attente d'avoir une nouvelle image et un process
				{
					compteur = 0;
					if(get_std_dev() > 18) //MAGIC NB
					{		
						chemin_trouve = TRUE; //path selected
					}
					else
					{
						recherche_chemin = FRONT;
						left_motor_set_pos(0);
						right_motor_set_pos(0);
					}
				}
			}
			break;

		case FRONT:

			*vitesse_rotation = -VITESSE_ROT_CHEMIN;
			if((abs(left_motor_get_pos()) >= 315) && (abs(right_motor_get_pos()) >= 315)) //MAGIC NB
			{
				*vitesse_rotation = 0;

				if(compteur++ >= 20) // attente d'avoir une nouvelle image et un process
				{
					compteur = 0;
					if(get_std_dev() > 18)  //MAGIC NB
					{
						recherche_chemin = RIGHT; //reset for next one
						chemin_trouve = TRUE; //path selected
					}
					else
						recherche_chemin = LEFT;
				}
			}
			break;

		case LEFT:

			*vitesse_rotation = -VITESSE_ROT_CHEMIN;
			if((abs(left_motor_get_pos()) >= 630) && (abs(right_motor_get_pos()) >= 630)) //MAGIC NB
			{
				*vitesse_rotation = 0;

				if(compteur++ >= 20) // attente d'avoir une nouvelle image et un process
				{
					compteur = 0;
					if(get_std_dev() > 18) //MAGIC NB
					{
						recherche_chemin = RIGHT; //reset for next one
						chemin_trouve = TRUE; //path selected
					}
					else
						recherche_chemin = BACK;
				}
			}
			break;

		case BACK:

			*vitesse_rotation = -VITESSE_ROT_CHEMIN;
			if((abs(left_motor_get_pos()) >= 980) && (abs(right_motor_get_pos()) >= 980)) //MAGIC NB
			{
				*vitesse_rotation = 0;

				if(compteur++ >= 20) // attente d'avoir une nouvelle image et un process
				{
					compteur = 0;

					recherche_chemin = RIGHT; //reset for next one
					chemin_trouve = TRUE; //selected path (by default)
				}
			}
			break;


		default:
			chprintf((BaseSequentialStream *)&SDU1, "MODE ERROR choix chemin");
			chemin_trouve = FALSE;
	}
	mode_inter_led(recherche_chemin);
	return chemin_trouve;
}

static THD_WORKING_AREA(waRob_management, 256);
static THD_FUNCTION(Rob_management, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    //a commenter plus tard
    systime_t time;

    int16_t speed = 0;
    int16_t speed_correction = 0;


    //demo varaiable is a state variable used to determine which part of the demonstration we are proceeding
    //DEMO1 : line alignment, robot moving through the maze, displacement algorithm, management of obstacles as walls
    //DEMO2 : same but now the robot can go through obstacles like a battering ram
    //DEMO3 : the synthesis of DEMO1/2, robot is using voice recognition to deal with osbstacles using go/back command
    static int8_t demo = DEMO1;

    //mode variable is a state variable used to adapt the epuck2 behavior
    static int8_t mode = NORMAL;

    gerer_led(NORMAL,1); //init les LEDS
 
    while(1){
        time = chVTGetSystemTime();
        
		//plus condens�s?
        static uint8_t compteur_int = 0;
        static uint8_t compteur_bl  = 0;
        static uint8_t compteur_dt  = 0;
        static uint8_t compteur_obst = 0;

        int8_t  vocal_command = 0;

        //Mode change, using selector	
    	demo = get_selector()%NB_DEMOS;

    	//MAGIC NB
        float ambient_light = (get_ambient_light(2)+get_ambient_light(5)+get_ambient_light(1)+get_ambient_light(6) + get_ambient_light(0) + get_ambient_light(7))/6;//magic nb

       //toujours besoin de std_dev?
        float std_dev = get_std_dev(); //performs standard deviation on the image from the camera
        _Bool intersection = (std_dev <= MAX_STD_INTER); //cross-roads detected
        _Bool blanc = ((std_dev > MAX_STD_INTER)&&(std_dev < MAX_STD_WHITE)); //end of the line

        //distance_mm gives the distance of the detected obstacle from the robot thanks to the TOF
        uint16_t distance_mm;
        distance_mm = VL53L0X_get_dist_mm();

        switch(mode)
        {
        	case NORMAL: //move forward following the line with a PID
        		 
        		speed = SPEED_DE_CROISIERE;
				speed_correction = pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, 0); // ne pas l'appeller tout le temps

        		//if the line is nearly in front of the camera, don't rotate
        		if(abs(speed_correction) < ROTATION_THRESHOLD || speed == 0){
        	       	speed_correction = 0;
        		}

        		//si distance est initialise (different de 0)
        		if(distance_mm && (distance_mm < GOAL_DISTANCE)) //si on est dans demo 2 et obstacle détecté on rentre en mode obstacle
        		{
        			compteur_bl = 0;
        			compteur_int = 0;
        			if(demo) mode = OBSTACLE;
        			else
        			{
        				left_motor_set_pos(0);
        				right_motor_set_pos(0);
        				pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, 1); //reset
        				mode = DEMI_TOUR;
        			}
        		}
        		else if (ambient_light > 3350 && blanc) //small ambient light and end of line
        		{
        			left_motor_set_pos(0);
        			right_motor_set_pos(0);
        			mode = END;
        		}
        		else if(intersection)
        		{
        			speed = 0;
        			speed_correction = 0;
        			if(compteur_int++ >= 10)
        			{
        				compteur_int = 0;
        				compteur_bl  = 0;
        				mode = INTERSECTION;
        				pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, 1); //reset
        	   			left_motor_set_pos(0);
              			right_motor_set_pos(0);
           			}
        		}
        		else if(blanc)
        		{
        			speed = 0;
        			speed_correction = 0;
        			if(compteur_bl++ >= 11) // attente d'avoir plusieurs valeurs de std
        			{
        				compteur_bl  = 0;
        				compteur_int = 0;
       					left_motor_set_pos(0);
        				right_motor_set_pos(0);
        				pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, 1); //reset
        				mode = DEMI_TOUR;
        			}

        		}
        		break;

        	case DEMI_TOUR: //performs a U-turn

				speed = 0;
				speed_correction = VITESSE_ROT_CHEMIN;

				if((abs(left_motor_get_pos()) >= 660) && (abs(right_motor_get_pos()) >= 660))
				{
					speed_correction = 0;

					if(compteur_dt++ >= 20) // attente d'avoir une nouvelle image et un process
					{
						compteur_dt = 0;
						mode = NORMAL;
					}
				}
				break;


        	case OBSTACLE: //deal with the presence of an obstacle

        		speed = -SPEED_DE_CROISIERE;
        		speed_correction = 0;
        		if(distance_mm >= DISTANCE_CHARGE)
        		{
        			speed = 0;
        			speed_correction = 0;
        			left_motor_set_pos(0);
        			right_motor_set_pos(0);
        			pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, 1);
        			if(demo = MODE2)
        				mode = ATTAQUE;
        			else
        			{
        				active_audio_processing();
        				if(compteur_obst++ > 20)
        				{
        					compteur_obst = 0;
        					vocal_command = return_vocal_command();
        					if(vocal_command != 0)
        					{
                				desactive_audio_processing();
        						if (vocal_command == GO)
        							mode = ATTAQUE;
        					    else if(vocal_command == C_BACK)
        					    {
        						   mode = DEMI_TOUR;
        						}
        					}
        				}
        			}
        		}
        		break;

        	case ATTAQUE: //increase the speed to kick a door

    			speed_correction = 0;
        		speed = VITESSE_CHARGE;
        		if(distance_mm >= 110 && left_motor_get_pos()>= 11*CONV_CM2STEP && right_motor_get_pos()>= 11*CONV_CM2STEP)
        		{

        			mode = NORMAL;
        		}
        		break;

        	case INTERSECTION: //detects a black square which represents future crossroads

        		speed = VITESSE_APPROCHE_INT;
        		speed_correction = 0;
        		if(left_motor_get_pos()>= 350 && right_motor_get_pos()>= 350) // 1.5*CONV_CM2STEP MAGIC NB
        		{
        			speed=0;
        			left_motor_set_pos(0);
        			right_motor_set_pos(0);
        			gerer_led_inter(RIGHT, 1); //init LEDs
        			mode = CHOIX_CHEMIN;
        		}
        		break;

        	case CHOIX_CHEMIN: //choose a correct path beetween 4 possible path in the priority: right, front, left, back

        		speed = 0;
        		if(choix_chemin(&speed_correction))
        		{
        			mode = NORMAL;
        			speed_correction = pi_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, 1); //reset
        		}
        		break;

        	case END: //detects the end of the demo, has entered his house!

        		speed = SPEED_DE_CROISIERE;
        		speed_correction = 0;
        		if(left_motor_get_pos()>= (350*1.5) && right_motor_get_pos()>= (350*1.5)) // 1.5*CONV_CM2STEP MAGIC NB
        		{
              		speed=0;
        		}
        		if(ambient_light < 3400) mode = NORMAL;
   				break;

        	default:
        		chprintf((BaseSequentialStream *)&SDU1, "MODE ERROR");
        }

        //change the states of the led, switch off previous mode LED and switch on new one 
       	mode_led(mode);

        //applies the speed from the PID regulator and the correction for the rotation
        right_motor_set_speed(speed - ROTATION_COEFF*speed_correction); //ROTATION_COEFF * à enlever si PID
        left_motor_set_speed(speed + ROTATION_COEFF*speed_correction); //ROTATION_COEFF * à enlever si PID

        //10Hz soit 100ms d'attente
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void rob_management_start(void){
	chThdCreateStatic(waRob_management, sizeof(waRob_management), NORMALPRIO+10, Rob_management, NULL);
}
