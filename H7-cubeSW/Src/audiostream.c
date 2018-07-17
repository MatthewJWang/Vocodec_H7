/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "codec.h"



// align is to make sure they are lined up with the data boundaries of the cache 
// at(0x3....) is to put them in the D2 domain of SRAM where the DMA can access them
// (otherwise the TX times out because the DMA can't see the data location) -JS


ALIGN_32BYTES (int16_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2);
ALIGN_32BYTES (int16_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2);

float detuneAmounts[NUM_VOICES];

#define TOTAL_BUFFERS 4

uint16_t* adcVals;

uint8_t buttonAPressed = 0;

uint8_t doAudio = 0;

uint16_t buffer_offset = 0;

float sample = 0.0f;
float noteperiod;
float adcx[8];
float pitchFactor;

float detuneMax = 16.0f;
uint8_t audioInCV = 0;
uint8_t audioInCVAlt = 0;
float myVol = 0.0f;

int lock;

float audioTickL(float audioIn); 
float audioTickR(float audioIn);
void buttonCheck(void);

tPitchShifter* ps;
tPitchShifter* ps2;

float inBuffer[4096];
float outBuffer[4096];
float outBuffer2[4096];

VocodecMode mode = PitchShiftMode;

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
	
	/* Initialize devices for pitch shifting */
	ps = tPitchShifter_init(HALF_BUFFER_SIZE);
	ps2 = tPitchShifter_init(HALF_BUFFER_SIZE);
	tPitchShifter_setPitchFactor(ps, 2.0f);
	tPitchShifter_setPitchFactor(ps2, 0.5f);
}



float tempVal = 0.0f;
uint16_t frameCounter = 0;
int numSamples = AUDIO_FRAME_SIZE;

void audioFrame(void)
{
	uint16_t i = 0;

	for (int cc=0; cc < numSamples; cc++)
	{
		inBuffer[(cur_read_block*numSamples)+cc] = (float) (audioInBuffer[buffer_offset+(cc*2)] * INV_TWO_TO_15 * 2);
	}

	tPitchShifter_ioSamples(ps, &inBuffer[cur_read_block*numSamples], &outBuffer[cur_write_block*numSamples], numSamples);
	tPitchShifter_ioSamples(ps2, &inBuffer[cur_read_block*numSamples], &outBuffer2[cur_write_block*numSamples], numSamples);

	for (int cc=0; cc < numSamples; cc++)
	{
		audioOutBuffer[buffer_offset + (cc*2)] = (int16_t) (outBuffer[cur_write_block*numSamples+cc] * TWO_TO_15);
		audioOutBuffer[buffer_offset + (cc*2)] += (int16_t) (outBuffer2[cur_write_block*numSamples+cc] * TWO_TO_15);
	}

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

void buttonWasPressed(VocodecButton button)
{
	int modex = (int) mode;

	if (button == ButtonUp)
	{
		modex++;
		if (modex >= ModeNil) modex = 3;
	}
	else if (button == ButtonDown)
	{
		modex--;
		if ((int)modex < 0) modex = 0;
	}
	else if (button == ButtonA)
	{

	}
	else if (button == ButtonB)
	{

	}

	mode = (VocodecMode) modex;

	writeModeToLCD(mode);
}

void buttonWasReleased(VocodecButton button)
{

}

#define BUTTON_HYSTERESIS 4
void buttonCheck(void)
{
	buttonValues[0] = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	buttonValues[1] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
	buttonValues[2] = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	buttonValues[3] = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);

	for (int i = 0; i < 4; i++)
	{
		if (buttonValues[i] != buttonValuesPrev[i])
		{
			if (buttonCounters[i] < BUTTON_HYSTERESIS)
			{
				buttonCounters[i]++;
			}
			else
			{
				if (buttonValues[i] == 1)
				{
					buttonPressed[i] = 1;
					buttonWasPressed(i);
				}
				else
				{
					buttonPressed[i] = 0;
					buttonWasReleased(i);
				}
				buttonValuesPrev[i] = buttonValues[i];
				buttonCounters[i] = 0;
			}
		}
	}
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

