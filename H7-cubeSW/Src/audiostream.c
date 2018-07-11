/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "codec.h"



// align is to make sure they are lined up with the data boundaries of the cache 
// at(0x3....) is to put them in the D2 domain of SRAM where the DMA can access them
// (otherwise the TX times out because the DMA can't see the data location) -JS


ALIGN_32BYTES (int16_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2);
ALIGN_32BYTES (int16_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2);

float detuneAmounts[NUM_OSC];

#define TOTAL_BUFFERS 4

uint16_t* adcVals;

uint8_t buttonAPressed = 0;

uint8_t doAudio = 0;

float sample = 0.0f;
float noteperiod;
float adcx[8];

float detuneMax = 16.0f;
uint8_t audioInCV = 0;
uint8_t audioInCVAlt = 0;
float myVol = 0.0f;

float audioTickL(float audioIn); 
float audioTickR(float audioIn);
void buttonCheck(void);

tSNAC* snac;
tSOLAD* sola;
tEnv* env;

tHighpass* hp;
tFormantShifter* fs;
tFormantShifter* fs2;
float inBuffer[4096];
float outBuffer[4096];



/* PSHIFT vars *************/
float lastmax;
int fba = 20;
int cur_read_block = 2, cur_write_block = 0;

float desPitchRatio = 2.0f;

float notePeriods[128];
int chordArray[12] = {1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0};

int hopSize = 64, windowSize = 64;
float max, timeConstant = 100, envout, deltamax, radius;
/**********************************************/


HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;


float nearestPeriod(float period)
{
	float leastDifference = fabsf(period - notePeriods[0]);
	float difference;
	int index;
	for(int i = 0; i < 128; i++)
	{
		if(chordArray[i%12] == 1)
		{
			difference = fabsf(period - notePeriods[i]);
			if(difference < leastDifference)
			{
				leastDifference = difference;
				index = i;
			}
		}
	}
	return notePeriods[index];
}

static void setTimeConstant(float tc)
{
    timeConstant = tc;
    radius = exp(-1000.0f * hopSize * oops.invSampleRate / timeConstant);
}

static BOOL attackDetect(void)
{
    envout = tEnvTick(env);

    if (envout >= 1.0f)
    {
        lastmax = max;
        if (envout > max)
        {
            max = envout;
        }
        else
        {
            deltamax = envout - max;
            max = max * radius;
        }
        deltamax = max - lastmax;
    }

    fba = fba ? (fba-1) : 0;

    if (fba == 0 && (max > 60 && deltamax > 6))
    {
        fba = 5;

        return TRUE;
    }
    return FALSE;
}


void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, RNG_HandleTypeDef* hrand, uint16_t* myADCArray)
{ 
	// Initialize the audio library. OOPS.
	OOPSInit(SAMPLE_RATE, HALF_BUFFER_SIZE, &randomNumber);
	
	for(int i = 0; i < 128; i++)
	{
		notePeriods[i] = 1.0f / OOPS_midiToFrequency(i);
	}
	fs = tFormantShifterInit();
	fs2 = tFormantShifterInit();
	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

	HAL_Delay(100);

	//poly = tPolyInit();
	adcVals = myADCArray;


	for (int i = 0; i < NUM_OSC; i++)
	{
		osc[i] = tSawtoothInit();
		tSawtoothSetFreq(osc[i], (randomNumber() * 2000.0f) + 40.0f);
		detuneAmounts[i] = (randomNumber() * detuneMax) - (detuneMax * 0.5f);
	}
	vocoder = tTalkboxInit();

	// set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);
	
	/* Initialize devices for pitch shifting */
	snac = tSNAC_init(HALF_BUFFER_SIZE, DEFOVERLAP);
	sola = tSOLAD_init();
	env = tEnvInit(windowSize, hopSize);

	hp = tHighpassInit(40.0f);

	tSOLAD_setPitchFactor(sola, 2.0f);

	setTimeConstant(timeConstant);


}



float tempVal = 0.0f;
uint16_t frameCounter = 0;
int numSamples = AUDIO_FRAME_SIZE;

void audioFrame(uint16_t buffer_offset)
{
	uint16_t i = 0;
	int16_t current_sample = 0;
	tSawtoothSetFreq(osc[0], 220.0f);
	for (int cc=0; cc < numSamples; cc++)
	{
		inBuffer[(cur_read_block*numSamples)+cc] = (float) (audioInBuffer[buffer_offset+(cc*2)] * INV_TWO_TO_15 * 4);
		//inBuffer[(cur_read_block*numSamples)+cc] = tSawtoothTick(osc[0]);
	}

	float  period, frequency, difference;

	tEnvProcessBlock(env, &inBuffer[cur_read_block*numSamples]);
	if (attackDetect()==TRUE)
	{
		tSOLAD_setReadLag(sola, oops.blockSize);
	}

	// tSNAC period detection
	tSNAC_ioSamples(snac, &inBuffer[cur_read_block*numSamples], &outBuffer[cur_write_block*numSamples], numSamples);

	period = tSNAC_getPeriod(snac);


	tSOLAD_setPeriod(sola, period);
	//adcVals[1]

	tSOLAD_setPitchFactor(sola, (period*oops.invSampleRate)/nearestPeriod(period*oops.invSampleRate));//(adcVals[1] * INV_TWO_TO_16) * 3.5f + 0.5f);
	//setTimeConstant((knobs[1] * INV_TWO_TO_12) * 20.0f + 480.0f);

	//  tSOLAD pshift works
	tFormantShifter_ioSamples(fs, &inBuffer[cur_read_block*numSamples], &outBuffer[cur_write_block*numSamples], numSamples, adcVals[1]*INV_TWO_TO_16*4.0f-2.0f);
	tFormantShifter_ioSamples(fs2, &outBuffer[cur_write_block*numSamples], &outBuffer[cur_write_block*numSamples], numSamples, adcVals[3]*INV_TWO_TO_16*4.0f-2.0f);
	//tSOLAD_ioSamples(sola, &outBuffer[cur_write_block*numSamples], &outBuffer[cur_write_block*numSamples], numSamples);

	for (int cc=0; cc < numSamples; cc++)
	{
		audioOutBuffer[buffer_offset + (cc*2)] = (int16_t) (tHighpassTick(hp,outBuffer[cur_write_block*numSamples+cc]) * TWO_TO_15);
		//audioOutBuffer[buffer_offset + (cc*2)] =  (int16_t) (inBuffer[(cur_read_block*numSamples)+cc] * TWO_TO_15); //inBuffer[(cur_read_block*numSamples)+cc]

	}

	cur_read_block++;
	if (cur_read_block >= TOTAL_BUFFERS)
		cur_read_block=0;

	cur_write_block++;
	if (cur_write_block >= TOTAL_BUFFERS)
		cur_write_block=0;


}

float currentFreq = 1.0f;

float rightInput = 0.0f;

float audioTickL(float audioIn) 
{
	sample = 0.0f;
	/*
	for (int i = 0; i < 4; i++)
	{
		tCycleSetFreq(mySine[i], ((((float)adcVals[i]) * INV_TWO_TO_16) * 1000.0f) + 40.0f);

	}
	*/

	for (int i = 0; i < NUM_OSC; i++)
	{
		if (tPolyGetMidiNote(poly, i)->on == OTRUE)
		{
			sample += tSawtoothTick(osc[i]);
		}
	}

	//sample = tSawtoothTick(osc[0]);
	//sample = audioIn;
	sample *= .25f;

	sample = tTalkboxTick(vocoder, sample, audioIn);
	sample = OOPS_softClip(sample, 0.98f);
	//sample *= myVol;
	//sample = audioIn;
	return sample;
}

float audioTickR(float audioIn) 
{
	rightInput = audioIn;
	//sample = audioIn;
	return audioIn;
}

void buttonCheck(void)
{
	buttonValues[0] = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
	buttonValues[1] = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
	buttonValues[2] = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
	buttonValues[3] = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);

	for (int i = 0; i < 4; i++)
	{
	  if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] < 40))
	  {
		  buttonCounters[i]++;
	  }
	  if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] >= 40))
	  {
		  if (buttonValues[i] == 1)
		  {
			  buttonPressed[i] = 1;
		  }
		  buttonValuesPrev[i] = buttonValues[i];
		  buttonCounters[i] = 0;
	  }
	}

	if (buttonPressed[0] == 1)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
		buttonPressed[0] = 0;

	}
	if (buttonPressed[1] == 1)
	{
		buttonPressed[1] = 0;
	}
	if (buttonPressed[2] == 1)
	{
		buttonPressed[2] = 0;
	}

	if (buttonPressed[3] == 1)
	{
		buttonPressed[3] = 0;
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
	audioFrame(HALF_BUFFER_SIZE);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);;
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	//doAudio = 1;
	audioFrame(0);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);;
}

