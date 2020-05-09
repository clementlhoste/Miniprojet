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

#define MIN_VALUE_THRESHOLD	10000 

//list of used frequencies for voice recognition
#define FREQ1_L			27	// 422 Hz
#define FREQ1_H			28	// 437 Hz
#define FREQ2			65 	//1016 Hz
#define FREQ3_L			97	//1515 Hz
#define FREQ3_H			98	//1531 Hz
#define FREQ4_L			137	//2140 Hz
#define FREQ4_H			138	//2156 Hz
#define FREQ5_L			201	//3140 Hz
#define FREQ5_H			202	//3156 Hz

//PNN parameters
#define NB_CLASSES		4
#define	NB_EXEMPLES		11
#define NB_FREQ			5
#define SMOOTHING		0.3f 
#define N1				1 //Nb of examples of class 1
#define N2				4 //Nb of examples of class 2
#define N3				3 //Nb of examples of class 3
#define N4				3 //Nb of examples of class 4

//needed to normalise input data
#define DATA_NORM		70000

// 4 classes, C_BACK to avoid conflict w/ direction in main.h
enum{VOID=1,SPEAK,GO,C_BACK};

//begins to analyse audio data (FFT + PNN)
void active_audio_processing(void);

//stops to analyse audio data (FFT + PNN)
void desactive_audio_processing(void);

//exports the result of analysis (0 = nothing, GO, C_BACK)
int8_t return_vocal_command(void);

//main analysis of sound (FFT/PNN)
void processAudioData(int16_t *data, uint16_t num_samples);

/*
*	Returns the pointer to the BUFFER_NAME_t buffer asked
*/
float* get_audio_buffer_ptr(BUFFER_NAME_t name);

#endif /* AUDIO_PROCESSING_H */
