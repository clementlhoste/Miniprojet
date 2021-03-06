#include "ch.h"
#include "hal.h"

#include <audio/microphone.h>
#include <audio_processing.h>
#include <arm_math.h>
#include <arm_const_structs.h>


///////////// CONSTANT DEFINES /////////////

//List of used frequencies for voice recognition
#define FREQ1_L			27	//  422 Hz
#define FREQ1_H			28	//  437 Hz
#define FREQ2			65 	// 1016 Hz the exact frequency needed
#define FREQ3_L			97	// 1515 Hz
#define FREQ3_H			98	// 1531 Hz
#define FREQ4_L			137	// 2140 Hz
#define FREQ4_H			138	// 2156 Hz
#define FREQ5_L			201	// 3140 Hz
#define FREQ5_H			202	// 3156 Hz

//PNN parameters
#define NB_CLASSES		4
#define	NB_EXEMPLES		11
#define NB_FREQ			5
#define SMOOTHING		0.3f	// Experimentally determined value
#define N1				1 		// Nb of examples of class 1
#define N2				4 		// Nb of examples of class 2
#define N3				3 		// Nb of examples of class 3
#define N4				3 		// Nb of examples of class 4

//Needed to normalize input data
#define DATA_NORM		70000


///////////// STATIC VARIABLES  /////////////

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];

//Examples of each class, used to feed the PNN, and to help to classify each input (FFT vector)
static const float examples[NB_EXEMPLES][NB_FREQ] = { {0.007694978,0.018722998,0.006465797,0.011846881,0.009480497}, 	//VOID
													  {0.025389122,0.022923238,0.008571391,0.013912240,0.011094523},	//SPEAK
													  {0.518886463,0.057486514,0.031562502,0.020052178,0.012833095},  	//SPEAK
													  {0.238694565,0.048119511,0.021042294,0.026486421,0.013906452},	//SPEAK
													  {0.077949966,0.023574965,0.009795453,0.013051499,0.010085111},	//SPEAK
													  {0.525953696,0.076893209,0.012715966,0.018951827,0.013572355},	//GO
													  {0.233461892,0.074033011,0.016969792,0.022156961,0.021261356},	//GO
													  {0.518886463,0.057486514,0.031562502,0.020052178,0.012833095},	//GO
													  {0.380157995,0.093555187,0.109508464,0.039939089,0.034523986}, 	//BACK
													  {0.175174589,0.058576963,0.033547497,0.026109697,0.015954384},	//BACK
													  {0.176715207,0.052898307,0.040889834,0.033202053,0.017457846} 	//BACK
											  		};

static _Bool process_active = FALSE;
static int8_t vocal_command  = 0;

///////////// INTERN FUNCTIONS /////////////

/*
*	Probabilistic Neural Network
*	This function was adapted from https://easyneuralnetwork.blogspot.com/2015/01/probabilistic-neural-network.html
*
*	params :
*	C is the number of classes, N is the number of examples, Nk are from class k
* 	d is the dimensionality of the training examples, sigma is the smoothing factor
* 	test_example[d] is the example to be classified
* 	Examples[N][d] are the training examples
*/
uint8_t pnn(uint8_t C, uint8_t N, uint8_t d, float sigma, float test_example[d], const float examples[N][d])
{
	uint8_t classify = -1;
	float largest = 0;
	float sum[C];
	uint8_t Nk = 0;
	uint8_t offset = 0;

	// The OUTPUT layer which computes the pdf for each class C
	for (uint8_t k=1; k<=C; k++)
	{
		sum[k] = 0;
		if(k == VOID)   Nk = N1;
		if(k == SPEAK)
		{
			Nk = N2;
			offset = N1; //should begin after N1 lines and do N2 lines
		}
		if(k == GO)
		{
			Nk = N3;
			offset = (N1+N2); //should begin after N1+N2 lines and do N3 lines
		}
		if(k == C_BACK)
		{
			Nk = N4;
			offset = (N1+N2+N3); //should begin after N1+N2+N3 lines and do N4 lines
		}
		// The SUMMATION layer which accumulates the pdf
		// for each example from the particular class k
		for (uint8_t i=0; i<Nk; i++)
		{
			float product = 0;
			// The PATTERN layer that multiplies the test example by the weights
			for (uint8_t j=0; j<d; j++)
			{
				product += (examples[i+offset][j] - test_example[j])*(examples[i+offset][j]-test_example[j]);
			}
			product = (-product) / (2* sigma * sigma);
			product = exp(product);
			sum[k] += product;
		}
		sum[k] /= Nk;
	}
	//OUTPUT layer which choose the largest "probability"
	for (uint8_t k=1; k<=C; k++)
	{
		if (sum[k] > largest)
		{
			largest = sum[k];
			classify = k;
		}
	}
	return classify;
}


/*
*	Allows to export FFT values, using the capture tool of Real Term, into a .txt
*	can be used directly in Excel to easily analyze data
*	Used to add examples of each class for the PNN
*	Need #include <chprintf.h> and USB serial config.
*	
*	params :
*	int16_t *data			Buffer containing FFT results

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
*/


///////////////// PUBLIC FUNCTIONS /////////////////

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*   FFT and PNN Analysis, to determine if a keyword (Go/Back) was pronounced
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
	
			//stops when buffer is full
			if(nb_samples >= (2 * FFT_SIZE)){
			break;
			}	
		}

		if(nb_samples >= (2 * FFT_SIZE)){
			/*	FFT processing
			*
			*	This FFT function stores the results in the input buffer given.
			*	Very optimized fft function provided by ARM
			*	which uses a lot of tricks to optimize the computations
			*/
			arm_cfft_f32(&arm_cfft_sR_f32_len1024, micLeft_cmplx_input, 0, 1);

			/*	Magnitude processing
			*
			*	Computes the magnitude of the complex numbers and
			*	stores them in a buffer of FFT_SIZE because it only contains
			*	real numbers.
			*
			*/
			arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

			//reset for the buffer
			nb_samples = 0;

			//this is the data we will give as input in the PNN, after normalization
			float test_example[NB_FREQ];

			//fills the test vector
			test_example[0] = (micLeft_output[FREQ1_L] + micLeft_output[FREQ1_H])/(2*DATA_NORM);//range of values around our frequency of interest
			test_example[1] = micLeft_output[FREQ2]/DATA_NORM;									//exact frequency, no mean needed
			test_example[2] = (micLeft_output[FREQ3_L] + micLeft_output[FREQ3_H])/(2*DATA_NORM);
			test_example[3] = (micLeft_output[FREQ4_L] + micLeft_output[FREQ4_H])/(2*DATA_NORM);
			test_example[4] = (micLeft_output[FREQ5_L] + micLeft_output[FREQ5_H])/(2*DATA_NORM);
		
			//we give parameters to PNN, plus an array with input FFT vector (test_example) to be classified
			//the example matrix contains examples of each class
			uint8_t class = 0;
			class = pnn(NB_CLASSES, NB_EXEMPLES, NB_FREQ, SMOOTHING, test_example, examples);

			//if a command is detected, we save it and we stop processing
			if(class == GO || class == C_BACK)
			{
				vocal_command  = class;
				process_active = FALSE;
			}
			else
			{
				vocal_command = 0;
			}
		}
	}
}


//exports the name of the buffers (only left mic used)
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

// begins the analysis of sound samples (FFT + PNN)
void active_audio_processing(void)
{
	process_active = TRUE;
}

// stops the analysis of sound samples (FFT + PNN)
void desactive_audio_processing(void)
{
	process_active = FALSE;
	vocal_command  = 0; //reset the vocal_cmd
}

//exports the result of the sound analysis (nothing or the command)
int8_t return_vocal_command(void)
{
	return vocal_command;
}
