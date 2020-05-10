#ifndef ROBOT_MANAGEMENT_H
#define ROBOT_MANAGEMENT_H

// Enum to define the different states of an FSM in which the robot can be
enum {NORMAL, OBSTACLE, ATTAQUE, INTERSECTION, CHOIX_CHEMIN, DEMI_TOUR, END};

// Enum to define the different directions the robot can take in an intersection
// and in which direction it is checking
enum {RIGHT, FRONT, LEFT, BACK};

// Enum to define the different demo modes
//DEMO1 : line alignment, robot moving through the maze, displacement algorithm, management of obstacles as walls
//DEMO2 : same but now the robot can go through obstacles like a battering ram
//DEMO3 : the synthesis of DEMO1/2, robot is using voice recognition to deal with osbstacles using go/back command
enum {DEMO1, DEMO2, DEMO3, NB_DEMOS};

#define ROTATION_THRESHOLD		10				// Do not rotafe if the error is small
#define ROTATION_COEFF			2				// Coefficient used to split PID action

//Constants for speed
#define SPEED_DE_CROISIERE		400				// [step/s] Speed when in mode normal
#define VITESSE_CHARGE			1000			// [step/s] Speed of the robot when it goes forward to break down an obstacle
#define VITESSE_APPROCHE_INT	200				// [step/s] Speed when approaching an intersection
#define VITESSE_ROT_CHEMIN    	250				// [step/s] Absolute value of speed for the 2 motors when
												// [step/s] rotating to choose a new path at an intersection
//Constants for the PID regulator
#define ERROR_THRESHOLD			5 				// [px]  because the camera is noisy (we don't want to take noise in consideration
												// during our robot alignment
#define KPL						0.5f  			// Parameters for line alignment PID : values determined according
#define KIL 				   	0.0018f	 		// to automatic lessons and experimental trials
#define KDL 				   	4.0f			//
#define TERM_I_MAX				200				// Maximum integral contribution to PID
#define MAX_SUM_ERROR_L			TERM_I_MAX/KIL  //limits sum error

//Camera and Proxi values
#define MAX_STD_INTER			10.7f			// Experimentally dertermined value
#define MAX_STD_WHITE			15.0f			// Experimentally dertermined value
#define MIN_STD_LINE			18.0f 			// Experimentally dertermined value
#define W_CALCUL_INT			20				// Wait counter
#define MIN_AMBIENT_L			3400			// Experimentally dertermined value
#define MAX_AMBIENT_L			3350			// Experimentally dertermined value
#define PROXI_R					2				// IR2 (right)
#define PROXI_L					5				// IR5 (left)
#define PROXI_FR45				1				// IR1 (front-right-45deg)
#define PROXI_FL45				6				// IR6 (front-left-45deg)
#define PROXI_FR				0				// IR0 (front-right)
#define PROXI_FL				7				// IR7 (front-left)
#define NB_PROXIS				6

//Distance and Rotation constants
#define PERIMETER_EPUCK			13                   // [cm]
#define CONV_CM2STEP			1000/PERIMETER_EPUCK // [step/cm]
#define GOAL_DISTANCE 			40					 // [mm] Detection distance of obstacles (doors)
#define RUN_DISTANCE			100 				 // [mm] Set-back distance before breaking down an obstacle (demo 2)
#define PAST_O_DISTANCE			RUN_DISTANCE+10		 // [mm] Distance where we know he have passed the obstacle
#define STEPS_INTER				350					 // [steps] Experimentally dertermined value
#define STEPS_HOUSE				1.5*STEPS_INTER	 	 // [steps] Experimentally dertermined value
#define STEPS_ATTAQUE			11*CONV_CM2STEP	     // [steps] Experimentally dertermined value
#define STEPS_U_TURN			660					 // [steps] Experimentally dertermined value for U turn
#define STEPS_Q_TURN			315					 // [steps] Experimentally dertermined value for Quarter turn (Intersection)
#define STEPS_BACK				980					 // [steps] Experimentally dertermined value the back direction in Intersection

//counters
#define CT_INTERSECTION			10				// help to be more precise in the color detection
#define CT_BLANC				11				// limits the error
#define CT_OBSTACLE				100				// waits in order to avoid the analysis of motor noise 

//Colors, RGB values
#define RED_R					200				// rgb(200,0,0)
#define ORANGE_R				255				// rgb(255,165,0)
#define ORANGE_G				165
#define BLUE_R					30				// rgb(30,144,255)
#define BLUE_G					144
#define BLUE_B					255

/*
*	Starts the Robot Management Thread
*/
void rob_management_start(void);

#endif /* ROBOT_MANAGEMENT_H */
