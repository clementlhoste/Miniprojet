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
#include <communications.h>
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

static void timer12_start(void){
    //General Purpose Timer configuration
    //timer 12 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt12cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD12, &gpt12cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD12, 0xFFFF);
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

	 //starts timer 12
	 timer12_start();

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
#ifdef DOUBLE_BUFFERING
    	//we copy the buffer to avoid conflicts
    	arm_copy_f32(get_audio_buffer_ptr(LEFT_OUTPUT), send_tab, FFT_SIZE);
    	SendFloatToComputer((BaseSequentialStream *) &SD3, send_tab, FFT_SIZE);
#else
    SendFloatToComputer((BaseSequentialStream *) &SD3, get_audio_buffer_ptr(LEFT_OUTPUT), FFT_SIZE);
#endif  /* DOUBLE_BUFFERING */
#else
        //time measurement variables
        volatile uint16_t time_fft = 0;
        volatile uint16_t time_mag  = 0;

        float* bufferCmplxInput = get_audio_buffer_ptr(LEFT_CMPLX_INPUT);
        float* bufferOutput = get_audio_buffer_ptr(LEFT_OUTPUT);

        uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, bufferCmplxInput, FFT_SIZE);

        if(size == FFT_SIZE){
            /*
            *   Optimized FFT
            */
            
            chSysLock();
            //reset the timer counter
            GPTD12.tim->CNT = 0;

            doFFT_optimized(FFT_SIZE, bufferCmplxInput);

            time_fft = GPTD12.tim->CNT;
            chSysUnlock();

            /*
            *   End of optimized FFT
            */

            /*
            *   Non optimized FFT
            */

            // //need to convert the float buffer into complex_float struct array
            // for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
            //     temp_tab[i/2].real = bufferCmplxInput[i];
            //     temp_tab[i/2].imag = bufferCmplxInput[i+1];
            // }

            // chSysLock();
            // //reset the timer counter
            // GPTD12.tim->CNT = 0;

            // //do a non optimized FFT
            // doFFT_c(FFT_SIZE, temp_tab);

            // time_fft = GPTD12.tim->CNT;
            // chSysUnlock();
            
            // //reconverts the result into a float buffer
            // for(uint16_t i = 0 ; i < (2*FFT_SIZE) ; i+=2){
            //     bufferCmplxInput[i] = temp_tab[i/2].real;
            //     bufferCmplxInput[i+1] = temp_tab[i/2].imag;
            // }

            /*
            *   End of non optimized FFT
            */

            chSysLock();
            //reset the timer counter
            GPTD12.tim->CNT = 0;

            arm_cmplx_mag_f32(bufferCmplxInput, bufferOutput, FFT_SIZE);

            time_mag = GPTD12.tim->CNT;
            chSysUnlock();

            SendFloatToComputer((BaseSequentialStream *) &SD3, bufferOutput, FFT_SIZE);
            //chprintf((BaseSequentialStream *) &SDU1, "time fft = %d us, time magnitude = %d us\n",time_fft, time_mag);

        }
#endif /* SEND_FROM_MIC */

    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
