#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>

#include <audio/microphone.h>
#include <audio_processing.h>
#include <fft.h>
#include <arm_math.h>

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];


#define MIN_VALUE_THRESHOLD	10000 

//list of used frequencies for voice recognition
#define FREQ1_L			27	//422Hz
#define FREQ1_H			28	//437Hz
#define FREQ2			65 	//1016 Hz
#define FREQ3_L			97	//1515 Hz
#define FREQ3_H			98	//1531 Hz
#define FREQ4_L			137	//2140 Hz
#define FREQ4_H			138	//2156 Hz
#define FREQ5_L			201	//3140 Hz
#define FREQ5_H			202	//3156 Hz

//PNN parameters
#define	NB_CLASSES		2
#define	NB_EXEMPLES		4
#define NB_FREQ			5
#define SMOOTHING		0.5f
#define N1				3
#define N2				1
enum{VOID=1,GO};

static float examples[NB_EXEMPLES][NB_FREQ] = { {0.007694978,0.018722998,0.006465797,0.011846881,0.009480497},
												{0.025389122,0.022923238,0.008571391,0.01391224,0.011094523},
												{0.077949966,0.023574965,0.009795453,0.013051499,0.010085111},
												//{0.056604504,0.033797138,0.007765485,0.014088029,0.01118684}
												{0.525953696,0.076893209,0.012715966,0.018951827,0.013572355}
											   };

static _Bool process_active = FALSE;
static _Bool vocal_command  = FALSE;

void active_audio_processing(void)
{
	process_active = TRUE;
}

void desactive_audio_processing(void)
{
	process_active = FALSE;
	vocal_command  = FALSE;
}

_Bool return_vocal_command(void)
{
	return vocal_command;
}

// C is the number of classes, N is the number of examples, Nk are from class k
// d is the dimensionality of the training examples, sigma is the smoothing factor
// test_example[d] is the example to be classified
// Examples[N][d] are the training examples
int pnn(uint8_t C, uint8_t N, uint8_t d, float sigma, float test_example[d], float examples[N][d])
{
	uint8_t classify = -1;
	float largest = 0;
	float sum[C];
	uint8_t Nk = 0;

	// The OUTPUT layer which computes the pdf for each class C
	for (uint8_t k=1; k<=C; k++)
	{
		sum[k] = 0;
		if(k == VOID)	Nk = N1;
		if(k == GO)		Nk = N2;

		// The SUMMATION layer which accumulates the pdf
		// for each example from the particular class k
		for (uint8_t i=0; i<Nk; i++)
		{
			if(k == GO)	i = N1;
			double product = 0;
			// The PATTERN layer that multiplies the test example by the weights
			for (uint8_t j=0; j<d; j++)
			{
				product += (examples[i][j] - test_example[j])*(examples[i][j]-test_example[j]);
			}
			//chprintf((BaseSequentialStream *) &SDU1, "#1 product (k %d,i %d) = %f \n;",k,i,product);
			product = (-product) / (2* sigma * sigma);
			//chprintf((BaseSequentialStream *) &SDU1, "#2 product (k %d,i %d) = %f \n;",k,i,product);
			product = exp(product);
			//chprintf((BaseSequentialStream *) &SDU1, "#3 product (k %d,i %d) = %f \n;",k,i,product);
			sum[k] += product;
		}
		sum[k] /= Nk;
	}
	for (uint8_t k=1; k<=C; k++)
	{
		//chprintf((BaseSequentialStream *) &SDU1, "probab %d: %f \n;",k,sum[k]);
		if (sum[k] > largest)
		{
			largest = sum[k];
			classify = k;
		}
	}
	return classify;
}

void extract_FFT(float* data)
{
	chprintf((BaseSequentialStream *) &SDU1, "%f;",data[FREQ1_L]);
	chprintf((BaseSequentialStream *) &SDU1, "%f;",data[FREQ1_H]);
	chprintf((BaseSequentialStream *) &SDU1, "%f;",data[FREQ2]);
	chprintf((BaseSequentialStream *) &SDU1, "%f;",data[FREQ3_L]);
	chprintf((BaseSequentialStream *) &SDU1, "%f;",data[FREQ3_H]);
	chprintf((BaseSequentialStream *) &SDU1, "%f; ",data[FREQ4_L]);
	chprintf((BaseSequentialStream *) &SDU1, "%f;",data[FREQ4_H]);
	chprintf((BaseSequentialStream *) &SDU1, "%f; ",data[FREQ5_L]);
	chprintf((BaseSequentialStream *) &SDU1, "%f \n",data[FREQ5_H]);
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/
	if(process_active)
	{
		static uint16_t nb_samples = 0;

		//loop to fill the buffers
		for(uint16_t i = 0 ; i < num_samples ; i+=4){
			//construct an array of complex numbers. Put 0 to the imaginary part
			micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

			nb_samples++;

			micLeft_cmplx_input[nb_samples] = 0;
	
			nb_samples++;
	
			//stop when buffer is full
			if(nb_samples >= (2 * FFT_SIZE)){
			break;
			}	
		}

		if(nb_samples >= (2 * FFT_SIZE)){
			/*	FFT proccessing
			*
			*	This FFT function stores the results in the input buffer given.
			*	This is an "In Place" function. 
			*/

			doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);

			/*	Magnitude processing
			*
			*	Computes the magnitude of the complex numbers and
			*	stores them in a buffer of FFT_SIZE because it only contains
			*	real numbers.
			*
			*/
			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

			nb_samples = 0;

			float test_example[NB_FREQ];
	
	
			test_example[0] = (micLeft_output[FREQ1_L] + micLeft_output[FREQ1_H])/(2*70000);
			test_example[1] = micLeft_output[FREQ2]/70000;
			test_example[2] = (micLeft_output[FREQ3_L] + micLeft_output[FREQ3_H])/(2*70000);
			test_example[3] = (micLeft_output[FREQ4_L] + micLeft_output[FREQ4_H])/(2*70000);
			test_example[4] = (micLeft_output[FREQ5_L] + micLeft_output[FREQ5_H])/(2*70000);
	
	
			uint8_t class = 0;
			class = pnn(NB_CLASSES, NB_EXEMPLES, NB_FREQ, SMOOTHING, test_example, examples);

			if(class == GO)
			{
				vocal_command  = TRUE;
				process_active = FALSE;
			}
		}
	}
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name)
{
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else{
		return NULL;
	}
}


