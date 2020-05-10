#include "ch.h"
#include "hal.h"

#include <led_gestion.h>
#include <robot_management.h>

///////////// CONSTANT DEFINES /////////////

//Colors, RGB values
#define RED_R					200		// rgb(200,0,0)
#define ORANGE_R				255		// rgb(255,165,0)
#define ORANGE_G				165
#define BLUE_R					30		// rgb(30,144,255)
#define BLUE_G					144
#define BLUE_B					255

///////////// PUBLIC FUNCTIONS /////////////

/*
*	LED change in the main finite state machine,
*   in order to better understand what does the robot
*
*	params :
*	int8_t   mode			state variable determining LED states
*   uint8_t  state    		1 = switch on led, 0 = switch off
*
*/
void gerer_led(int8_t mode, uint8_t state)
{
	switch(mode)
	{
	  	case NORMAL:
        //switch the front LED
        	set_led(LED1, state);
        	break;

	    case DEMI_TOUR:
	    //switch 2 back RGB LED (in RED here)
			set_rgb_led(LED4,(state * RED_R),0,0);
			set_rgb_led(LED6,(state * RED_R),0,0);
			break;

	    case OBSTACLE:
	    //switch back LED
	     	set_led(LED5, state);
	       	break;

	    case ATTAQUE:
	    //switch 4 RGB LED in orange
	      	set_rgb_led(LED2,(state*ORANGE_R),(state*ORANGE_G),0);
	        set_rgb_led(LED4,(state*ORANGE_R),(state*ORANGE_G),0);
         	set_rgb_led(LED6,(state*ORANGE_R),(state*ORANGE_G),0);
	        set_rgb_led(LED8,(state*ORANGE_R),(state*ORANGE_G),0);
	       	break;

        case INTERSECTION:
	    //switch 4 RGB LED in blue
	        set_rgb_led(LED2,(state*BLUE_R),(state*BLUE_G),(state*BLUE_B));
	        set_rgb_led(LED4,(state*BLUE_R),(state*BLUE_G),(state*BLUE_B));
         	set_rgb_led(LED6,(state*BLUE_R),(state*BLUE_G),(state*BLUE_B));
	        set_rgb_led(LED8,(state*BLUE_R),(state*BLUE_G),(state*BLUE_B));
	        break;

	    case CHOIX_CHEMIN:
	    //only possible to switch off this light, on is in gerer_led_inter
	     	set_led(LED3, 0);
	   		break;

	    default: //END or any unknown case
	 		//do nothing
	    	break;
	}
}

/*
*	LED change management for the main finite state machine
*   in order to better understand what does the robot
*
*	params :
*	int8_t   mode			state variable determining LED states
*
*/
void mode_led(int8_t mode)
{
	static int8_t ancien_mode = NORMAL;

	if(mode != ancien_mode)
	{
		//switch off last "mode" corresponding LEDs
		gerer_led(ancien_mode, 0);

		if(mode != CHOIX_CHEMIN) //case corresponding to gerer_led_inter()
		{
			//switch on new "mode" corresponding LEDs
			gerer_led(mode, 1);
		}
		ancien_mode = mode;
	}
}

/*
*	LED change in the direction finite state machine,
*   in order to better understand where the robot goes
*
*	params :
*	int8_t   dir			    state variable determining LED states
*   uint8_t  state    			1 = switch on led, 0 = switch off
*
*/
void gerer_led_inter(int8_t dir, uint8_t state)
{
	switch(dir)
	{
	  	case RIGHT:
        //switch the front LED
        	set_led(LED3, state);
        	break;

	    case FRONT:
	    //switch 2 front RGB LED (in RED here)
			set_rgb_led(LED2,(state*RED_R),0,0);
			set_rgb_led(LED8,(state*RED_R),0,0);
			break;

	    case LEFT:
	    //switch left LED
	     	set_led(LED7,state);
	       	break;

	    case BACK:
	    //switch back LED
	   		set_led(LED5,state);
	       	break;

	    default: //any unknown case
	 		//do nothing
	    	break;
	}
}

/*
*	LED change management in the direction finite state machine,
*   in order to better understand where the robot goes
*
*	params :
*	int8_t   dir			state variable determining LED states
*
*/
void mode_inter_led(int8_t dir)
{
	static uint8_t ancienne_dir = RIGHT;

	if(dir != ancienne_dir)
	{
		//switch off last "dir" corresponding LEDs
		gerer_led_inter(ancienne_dir, 0);
		//switch on new "dir" corresponding LEDs
		gerer_led_inter(dir, 1);
	}
	ancienne_dir = dir;
}
