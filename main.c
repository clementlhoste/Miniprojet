#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <camera/po8030.h>
#include <chprintf.h>
#include <spi_comm.h>
#include <audio/microphone.h>
#include <arm_math.h>

#include <process_image.h>
#include <audio_processing.h>
#include <robot_management.h>

// Necessary declarations to use the proximity sensors
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

// Function called in main function to initialize the serial communication (DELETE?)
static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication (to DELETE)
    serial_start();
    //starts the USB communication (to DELETE)
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();

	//initialization necessary for the use of proximity sensors
	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//inits the motors
	motors_init();
	//allows us to use the RGB leds
	spi_comm_start();

	//starts the ToF Thread
	VL53L0X_start();

	//start the proximity sensors
	proximity_start();
	calibrate_ir();

	//starts the threads for the robot management and the processing of the image in order to follow lines,
	//intersection detection and end of lines
	process_image_start();
	rob_management_start();

	//send_tab is used to save the state of the buffer to send (double buffering)
	//to avoid modifications of the buffer while sending it //
	static float send_tab[FFT_SIZE];


	 //starts the microphones processing thread.
	 //it calls the callback given in parameter when samples are ready
	 mic_start(&processAudioData);


    /* Infinite loop. */
    while (1) {

    //we copy the buffer to avoid conflicts
    	arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);

    	//waits 1 second
 	   	chThdSleepMilliseconds(1000);

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
