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

#include <process_image.h>
#include <robot_management.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

//uncomment to send the FFTs results from the real microphones
#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
#define DOUBLE_BUFFERING

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void SendUint8ToComputer(uint8_t* data, uint16_t size) 
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

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

    //starts the serial communication
    serial_start();
    //start the USB communication
    usb_start();
    //starts the camera
    dcmi_start();
	po8030_start();

	messagebus_init(&bus, &bus_lock, &bus_condvar);

	//inits the motors
	motors_init();
	//allows us to use the rgb leds
	spi_comm_start();

	//starts the ToF Thread
	VL53L0X_start();

	//start the proximity sensors
	proximity_start();
	calibrate_ir();

	//starts the threads for the pi regulator and the processing of the image
	process_image_start();
	rob_management_start();

	//temp tab used to store values in complex_float format
	//needed bx doFFT_c
	static complex_float temp_tab[FFT_SIZE];
	//send_tab is used to save the state of the buffer to send (double buffering)
	//to avoid modifications of the buffer while sending it
	static float send_tab[FFT_SIZE];

	#ifdef SEND_FROM_MIC
	    //starts the microphones processing thread.
	    //it calls the callback given in parameter when samples are ready
	    mic_start(&processAudioData);
	#endif  /* SEND_FROM_MIC */



    /* Infinite loop. */
    while (1) {
		#ifdef SEND_FROM_MIC

    		//waits until a result must be sent to the computer
    		wait_send_to_computer();
		#endif
        #ifdef DOUBLE_BUFFERING
    		//we copy the buffer to avoid conflicts
    		arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
    		//SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
		#endif

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
