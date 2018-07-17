/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "codec.h"

// align is to make sure they are lined up with the data boundaries of the cache 
// at(0x3....) is to put them in the D2 domain of SRAM where the DMA can access them
// (otherwise the TX times out because the DMA can't see the data location) -JS
ALIGN_32BYTES (int16_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2);
ALIGN_32BYTES (int16_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2);

#define TOTAL_BUFFERS 4

uint16_t* adcVals;

uint16_t buffer_offset = 0;

float sample = 0.0f;;

float audioTickL(float audioIn); 
float audioTickR(float audioIn);

#define NUM_SHIFTERS 3

tPitchShifter* ps[NUM_SHIFTERS];

float pitchFactors[] = { 1.2f, 1.5f, 2.0f };

float inBuffer[4096];
float outBuffer[4096];

int cur_read_block = 2, cur_write_block = 0;
/**********************************************/

HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, RNG_HandleTypeDef* hrand, uint16_t* myADCArray)
{ 
	// Initialize the audio library. OOPS.
	OOPSInit(SAMPLE_RATE, HALF_BUFFER_SIZE, &randomNumber);

	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

	HAL_Delay(100);

	adcVals = myADCArray;

	// set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);

	/* Initialize for pitch shifting devices and set pitch factors */
	for (int i = 0; i < NUM_SHIFTERS; ++i)
	{
		ps[i] = tPitchShifter_init(HALF_BUFFER_SIZE);
		tPitchShifter_setPitchFactor(ps[i], pitchFactors[i]);
	}
}

int numSamples = AUDIO_FRAME_SIZE;

void audioFrame(void)
{
	uint16_t i = 0;

	for (int cc=0; cc < numSamples; cc++)
	{
		inBuffer[(cur_read_block*numSamples)+cc] = (float) (audioInBuffer[buffer_offset+(cc*2)] * INV_TWO_TO_15 * 2);
	}

	/***** Pitchshifting *************************/

	for (int i = 0; i < NUM_SHIFTERS; ++i)
	{
		// Set pitch ratio each audio frame
		tPitchShifter_setPitchFactor(ps[i], (adcVals[1] * INV_TWO_TO_16)*pitchFactors[i]);

		// Pitch shift the input buffer and place into the output buffer
		tPitchShifter_ioSamples(ps[i], &inBuffer[cur_read_block*numSamples], &outBuffer[cur_write_block*numSamples], numSamples);

		// Add the output buffer to the audio output
		for (int cc=0; cc < numSamples; cc++)
		{
			if (i == 0) audioOutBuffer[buffer_offset + (cc*2)] = (int16_t) (outBuffer[cur_write_block*numSamples+cc] * TWO_TO_15);
			else audioOutBuffer[buffer_offset + (cc*2)] += (int16_t) (outBuffer[cur_write_block*numSamples+cc] * TWO_TO_15);
		}
	}

	/*********************************************/

	cur_read_block++;
	if (cur_read_block >= TOTAL_BUFFERS)
		cur_read_block=0;

	cur_write_block++;
	if (cur_write_block >= TOTAL_BUFFERS)
		cur_write_block=0;
}

float audioTickL(float audioIn) 
{
	sample = 0.0f;

	return sample;
}

float audioTickR(float audioIn) 
{
	sample = 0.0f;

	return sample;
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	;
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  ;
}


void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	//doAudio = 2;
	buffer_offset = HALF_BUFFER_SIZE;
	audioFrame();
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);;
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	//doAudio = 1;
	buffer_offset = 0;
	audioFrame();
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);;
}

