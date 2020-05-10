#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H

#define FFT_SIZE 	1024

typedef enum {
	//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
	LEFT_CMPLX_INPUT = 0,
	RIGHT_CMPLX_INPUT,
	FRONT_CMPLX_INPUT,
	BACK_CMPLX_INPUT,
	//Arrays containing the computed magnitude of the complex numbers
	LEFT_OUTPUT,
	RIGHT_OUTPUT,
	FRONT_OUTPUT,
	BACK_OUTPUT
} BUFFER_NAME_t;

// 4 classes, C_BACK to avoid conflict w/ direction in robot_management.h
enum{VOID=1,SPEAK,GO,C_BACK};

/*
*	Begins to analyze audio data (FFT + PNN) in processAudioData
*/
void active_audio_processing(void);

/*
*	Stops to analyze audio data (FFT + PNN) in processAudioData
*/
void desactive_audio_processing(void);

/*
*	Exports the result of analysis (0 = nothing, GO, C_BACK)
*/
int8_t return_vocal_command(void);

/*
*	Callback function, main analysis of sound (FFT/PNN)
*
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

#endif /* AUDIO_PROCESSING_H */
