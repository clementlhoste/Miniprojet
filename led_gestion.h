#ifndef LED_GESTION_H_
#define LED_GESTION_H_

#include "../lib/e-puck2_main-processor/src/leds.h"

/*
*	LED change in the main finite state machine,
*   in order to better understand what does the robot
*
*	params :
*	int8_t   mode			state variable determining LED states
*   uint8_t  state    		1 = switch on led, 0 = switch off
*
*/
void gerer_led(int8_t mode, uint8_t state);

/*
*	LED change management for the main finite state machine
*   in order to better understand what does the robot
*
*	params :
*	int8_t   mode			state variable determining LED states
*
*/
void mode_led(int8_t mode);

/*
*	LED change in the direction finite state machine,
*   in order to better understand where the robot goes
*
*	params :
*	int8_t   dir			    state variable determining LED states
*   uint8_t  state    			1 = switch on led, 0 = switch off
*
*/
void gerer_led_inter(int8_t dir, uint8_t state);

/*
*	LED change management in the direction finite state machine,
*   in order to better understand where the robot goes
*
*	params :
*	int8_t   dir			state variable determining LED states
*
*/
void mode_inter_led(int8_t dir);

#endif /* LED_GESTION_H_ */
