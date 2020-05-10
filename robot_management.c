#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h> //DELETE?
#include <chprintf.h> //DELETE?
#include <main.h>
#include <led_gestion.h>
#include <process_image.h>
#include <audio_processing.h>
#include <robot_management.h>
#include "../lib/e-puck2_main-processor/src/selector.h"


///////////// CONSTANT DEFINES /////////////

#define ROTATION_THRESHOLD		10				// Do not rotate if the error is small
#define ROTATION_COEFF			2				// Coefficient used to split PID action

//Constants for speed
#define SPEED_DE_CROISIERE		400				// [step/s] Speed when in mode normal
#define VITESSE_CHARGE			1000				// [step/s] Speed of the robot when it goes forward to break down an obstacle
#define VITESSE_APPROCHE_INT		200				// [step/s] Speed when approaching an intersection
#define VITESSE_ROT_CHEMIN    	250				// [step/s] Absolute value of speed for the 2 motors when
												// [step/s] rotating to choose a new path at an intersection
//Constants for the PID regulator
#define ERROR_THRESHOLD			5 				// [px]  because the camera is noisy (we don't want to take noise in consideration
												// during our robot alignment
#define KPL						0.5f  			// Parameters for line alignment PID : values determined according
#define KIL 				  	 	0.0018f	 		// to automatic lessons and experimental trials
#define KDL 				   		4.0f				//
#define TERM_I_MAX				200				// Maximum integral contribution to PID
#define MAX_SUM_ERROR_L			TERM_I_MAX/KIL  //limits sum error

//Camera and Proxi values
#define MAX_STD_INTER			10.7f			// Experimentally determined value
#define MAX_STD_WHITE			15.0f			// Experimentally determined value
#define MIN_STD_LINE				18.0f 			// Experimentally determined value
#define W_CALCUL_INT				20				// Wait counter
#define MIN_AMBIENT_L			3400				// Experimentally determined value
#define MAX_AMBIENT_L			3350				// Experimentally determined value
#define PROXI_R					2				// IR2 (right)
#define PROXI_L					5				// IR5 (left)
#define PROXI_FR45				1				// IR1 (front-right-45deg)
#define PROXI_FL45				6				// IR6 (front-left-45deg)
#define PROXI_FR					0				// IR0 (front-right)
#define PROXI_FL					7				// IR7 (front-left)
#define NB_PROXIS				6

//Distance and Rotation constants
#define PERIMETER_EPUCK			13                    // [cm]
#define CONV_CM2STEP				1000/PERIMETER_EPUCK  // [step/cm]
#define GOAL_DISTANCE 			40					 // [mm] Detection distance of obstacles (doors)
#define RUN_DISTANCE				100 					 // [mm] Set-back distance before breaking down an obstacle (demo 2)
#define PAST_O_DISTANCE			RUN_DISTANCE+10		 // [mm] Distance where we know he have passed the obstacle
#define STEPS_INTER				350					 // [steps] Experimentally determined value
#define STEPS_HOUSE				1.5*STEPS_INTER	 	 // [steps] Experimentally determined value
#define STEPS_ATTAQUE			11*CONV_CM2STEP	     // [steps] Experimentally determined value
#define STEPS_U_TURN				660					 // [steps] Experimentally determined value for U turn
#define STEPS_Q_TURN				315					 // [steps] Experimentally determined value for Quarter turn (Intersection)
#define STEPS_BACK				980					 // [steps] Experimentally determined value the back direction in Intersection

//counters
#define CT_INTERSECTION			10				// help to be more precise in the color detection
#define CT_BLANC					11				// limits the error
#define CT_OBSTACLE				100				// waits in order to avoid the analysis of motor noise

///////////// INTERN FUNCTIONS /////////////

/*
*	PID regulator implementation to manage line alignment
*	
*	params :
*	uint16_t distance		position	
*   uint16_t goal    		command (goal position)
*   _Bool    reset 			used to reset sum_error (reset I contribution)		
*   
*/
int16_t pid_regulator(uint16_t distance, uint16_t goal, _Bool reset){

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


	//disables the PID regulator if the error is to small
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

	//activate the Kd coefficient only if error_pre exists
	if(error_pre)
		speed = KPL * error + KIL * sum_error + KDL*(error-error_pre);
	else
		speed = KPL * error + KIL * sum_error;

	error_pre = error;
	
    return speed;
}


/*
*	Direction Finite State Machine
*	Find the first path available in the order of priority: 
*   Right, Front, Left, Back
*	
*	params :
*	int16_t*   vitesse_rotation		decide direction/speed		
*   
*/
_Bool choix_chemin(int16_t* vitesse_rotation)
{

	//state variable indicating the direction of the robot (RIGHT/FRONT/LEFT/BACK)
	static int8_t recherche_chemin = RIGHT;
	_Bool chemin_trouve = FALSE;
	static uint8_t compteur = 0;

	switch(recherche_chemin)
	{
		case RIGHT: //At first, check if there is a path on the right

			*vitesse_rotation = VITESSE_ROT_CHEMIN;

			if((abs(left_motor_get_pos()) >= STEPS_Q_TURN) && (abs(right_motor_get_pos()) >= STEPS_Q_TURN)) //quarter turn
			{
				*vitesse_rotation = 0;
				if(compteur++ >= W_CALCUL_INT) //wait to get new calculations
				{
					compteur = 0;
					if(get_std_dev() > MIN_STD_LINE)
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

		case FRONT: //if not, check front

			*vitesse_rotation = -VITESSE_ROT_CHEMIN;
			if((abs(left_motor_get_pos()) >= STEPS_Q_TURN) && (abs(right_motor_get_pos()) >= STEPS_Q_TURN)) //quarter turn, change dir
			{
				*vitesse_rotation = 0;

				if(compteur++ >= W_CALCUL_INT) //wait to get new calculations
				{
					compteur = 0;
					if(get_std_dev() > MIN_STD_LINE) 
					{
						recherche_chemin = RIGHT; //reset for next one
						chemin_trouve = TRUE; //path selected
					}
					else
						recherche_chemin = LEFT;
				}
			}
			break;

		case LEFT: //if not check left

			*vitesse_rotation = -VITESSE_ROT_CHEMIN;
			if((abs(left_motor_get_pos()) >= (STEPS_Q_TURN*2)) && (abs(right_motor_get_pos()) >= (STEPS_Q_TURN*2))) //new quarter turn
			{
				*vitesse_rotation = 0;

				if(compteur++ >= W_CALCUL_INT) //wait to get new calculations
				{
					compteur = 0;
					if(get_std_dev() > MIN_STD_LINE)
					{
						recherche_chemin = RIGHT; //reset for next one
						chemin_trouve = TRUE; //path selected
					}
					else
						recherche_chemin = BACK;
				}
			}
			break;

		case BACK: //no path found, U turn

			*vitesse_rotation = -VITESSE_ROT_CHEMIN;
			if((abs(left_motor_get_pos()) >= STEPS_BACK) && (abs(right_motor_get_pos()) >= STEPS_BACK)) //approx 3 quarter turn (exp. value)
			{
				*vitesse_rotation = 0;

				if(compteur++ >= W_CALCUL_INT) //wait to get new calculations
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
	//change LED depending on the direction
	mode_inter_led(recherche_chemin);

	return chemin_trouve;
}

///////////// THREAD /////////////

/*
*	Main Finite State Machine
*   Determines the robot behavior depending on its environment
*
*	params :
*   -
*   
*/
static THD_WORKING_AREA(waRob_management, 256);
static THD_FUNCTION(Rob_management, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed = 0, speed_correction = 0;
    uint16_t distance_mm;       //distance of the detected obstacle from the robot thanks to the TOF
	int8_t  vocal_command = 0;  // 0/GO/C_BACK
	float ambient_light = 0.0, std_dev = 0.0;
	_Bool intersection = FALSE, blanc = FALSE;

    //demo variable is a state variable used to determine which part of the demonstration we are proceeding
    static int8_t demo = DEMO1;

    //mode variable is a state variable used to adapt the epuck2 behavior
    static int8_t mode = NORMAL;

    static uint8_t compteur_int = 0, compteur_bl  = 0, compteur_dt  = 0, compteur_obst = 0;

    gerer_led(NORMAL,1); //LED initialization
 
    while(1){

        time = chVTGetSystemTime();

        //Mode change, using selector	
    		demo = get_selector()%NB_DEMOS;

        ambient_light = (get_ambient_light(PROXI_R)+get_ambient_light(PROXI_L)+get_ambient_light(PROXI_FR45)
        				+get_ambient_light(PROXI_FL45) + get_ambient_light(PROXI_FR) + get_ambient_light(PROXI_FL))/NB_PROXIS;

        distance_mm = VL53L0X_get_dist_mm();

        switch(mode)
        {
        	case NORMAL: //move forward following the line with a PID
        		 
        		std_dev = get_std_dev(); //performs standard deviation on the image from the camera
        		intersection = (std_dev <= MAX_STD_INTER); //cross-roads detected
       			blanc = ((std_dev > MAX_STD_INTER)&&(std_dev < MAX_STD_WHITE)); //end of the line

        		//if the distance is well initialized (not 0)
        		if(distance_mm && (distance_mm < GOAL_DISTANCE)) //DEMO 2 and obsctale detected
        		{
        			compteur_bl = 0;
        			compteur_int = 0;
        			speed = 0;
        			speed_correction = 0;
        			if(demo) mode = OBSTACLE;
        			else
        			{
        				left_motor_set_pos(0);
        				right_motor_set_pos(0);
        				pid_regulator(0, 0, 1); //reset sum_error term
        				mode = DEMI_TOUR;
        			}
        		}
        		else if (ambient_light > MAX_AMBIENT_L && blanc) //small ambient light (inverse scale) and end of line
        		{
        			speed = 0;
        			speed_correction = 0;
        			left_motor_set_pos(0);
        			right_motor_set_pos(0);
        			mode = END;
        		}
        		else if(intersection) //crossroad detected
        		{
        			speed = 0;
        			speed_correction = 0;
        			if(compteur_int++ >= CT_INTERSECTION) // be more precise in the detection (multiple values)
        			{
        				compteur_int = 0;
        				compteur_bl  = 0;
        				pid_regulator(0, 0, 1); //reset sum_error term
        	   			left_motor_set_pos(0);
              			right_motor_set_pos(0);
              			mode = INTERSECTION;
           			}
        		}
        		else if(blanc) //end of line detected
        		{
        			speed = 0;
        			speed_correction = 0;
        			if(compteur_bl++ >= CT_BLANC) // be more precise in the detection (multiple values)
        			{
        				compteur_bl  = 0;
        				compteur_int = 0;
       					left_motor_set_pos(0);
        				right_motor_set_pos(0);
        				pid_regulator(0, 0, 1); //reset sum_error term
        				mode = DEMI_TOUR;
        			}

        		}
        		else //stay in NORMAL and apply pid regulation
        		{
        			speed = SPEED_DE_CROISIERE;
					speed_correction = pid_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, 0);

        			//if the line is nearly in front of the camera, don't rotate
        			if(abs(speed_correction) < ROTATION_THRESHOLD)
        			{
        	       		speed_correction = 0;
        			}
        		}
        		break;

        	case DEMI_TOUR: //performs a U-turn

				speed = 0;
				speed_correction = VITESSE_ROT_CHEMIN;

				if((abs(left_motor_get_pos()) >= STEPS_U_TURN) && (abs(right_motor_get_pos()) >= STEPS_U_TURN))
				{
					speed_correction = 0;

					if(compteur_dt++ >= W_CALCUL_INT) // wait to get new caclulations (little break)
					{
						compteur_dt = 0;
						mode = NORMAL;
					}
				}
				break;


        	case OBSTACLE: //deal with the presence of an obstacle

        		speed = -SPEED_DE_CROISIERE;
        		speed_correction = 0;
        		if(distance_mm >= RUN_DISTANCE || compteur_obst) //avoid distance noise in vocal command
        		{
        			speed = 0;
        			speed_correction = 0;
        			left_motor_set_pos(0);
        			right_motor_set_pos(0);
        			pid_regulator(get_line_position(), IMAGE_BUFFER_SIZE/2, 1);
        			if(demo == DEMO2)
        				mode = ATTAQUE;
        			else //mode DEMO3
        			{
        				active_audio_processing(); //voice recognition enabled
        				if(compteur_obst++ > CT_OBSTACLE)   //avoid motor noise
        				{
        					set_led(LED1,1);
        					vocal_command = return_vocal_command();
        					if(vocal_command != 0)
        					{
        						compteur_obst = 0;
        						desactive_audio_processing();
        						if (vocal_command == GO)
        							mode = ATTAQUE;
        					    else if(vocal_command == C_BACK)
        						   mode = DEMI_TOUR;
        					}
        				}
        			}
        		}
        		break;

        	case ATTAQUE: //increase the speed to kick a door

    			speed_correction = 0;
        		speed = VITESSE_CHARGE;
        		if(distance_mm >= PAST_O_DISTANCE && left_motor_get_pos()>= (STEPS_ATTAQUE) && right_motor_get_pos()>= (STEPS_ATTAQUE))
        		{

        			mode = NORMAL;
        		}
        		break;

        	case INTERSECTION: //detects a black square which represents future crossroads

        		speed = VITESSE_APPROCHE_INT;
        		speed_correction = 0;
        		if(left_motor_get_pos()>= STEPS_INTER && right_motor_get_pos()>= STEPS_INTER)
        		{
        			speed = 0;
        			left_motor_set_pos(0);
        			right_motor_set_pos(0);
        			gerer_led_inter(RIGHT, 1); //LEDs init
        			mode = CHOIX_CHEMIN;
        		}
        		break;

        	case CHOIX_CHEMIN: //choose a correct path beetween 4 possible path in the priority: right, front, left, back

        		speed = 0;
        		if(choix_chemin(&speed_correction))
        		{
        			mode = NORMAL;
        			pid_regulator(0, 0, 1); //reset sum_error term
        		}
        		break;

        	case END: //detects the end of the demo, robot has entered his house!

        		speed = SPEED_DE_CROISIERE;
        		speed_correction = 0;
        		if(left_motor_get_pos()>= (STEPS_HOUSE) && right_motor_get_pos()>= (STEPS_HOUSE))
        		{
              		speed=0;
        		}
        		if(ambient_light < MIN_AMBIENT_L) mode = NORMAL; //if high ambient light (inverse scale)
   				break;

        	default:
        		chprintf((BaseSequentialStream *)&SDU1, "MODE ERROR");
        }

        //change the states of the led, switch off previous mode LED and switch on new one 
       	mode_led(mode);

        //applies the speed from the PID regulator and the correction for the rotation
        right_motor_set_speed(speed - ROTATION_COEFF*speed_correction);
        left_motor_set_speed(speed + ROTATION_COEFF*speed_correction);

        //10Hz so waits 100ms
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}


///////////// PUBLIC FUNCTIONS /////////////

/*
*	Starts the Robot Management Thread
*/
void rob_management_start(void){
	chThdCreateStatic(waRob_management, sizeof(waRob_management), NORMALPRIO+10, Rob_management, NULL);
}
