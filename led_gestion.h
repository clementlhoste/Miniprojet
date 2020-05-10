#ifndef LED_GESTION_H_
#define LED_GESTION_H_

#include "../lib/e-puck2_main-processor/src/leds.h"

void gerer_led(int8_t mode, uint8_t state);

void mode_led(int8_t mode);

void gerer_led_inter(int8_t dir, uint8_t state);

void mode_inter_led(int8_t dir);

#endif /* LED_GESTION_H_ */
