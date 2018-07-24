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

float detuneMax = 16.0f;
uint8_t audioInCV = 0;
uint8_t audioInCVAlt = 0;
float myVol = 0.0f;

int lock;

float audioTickL(float audioIn); 
float audioTickR(float audioIn);
void buttonCheck(void);

float inBuffer[NUM_SHIFTERS][4096];
float outBuffer[NUM_SHIFTERS][4096];

tFormantShifter* fs;
tFormantShifter* fs2;
tPitchShifter* ps[NUM_SHIFTERS];
tRamp* ramp[NUM_VOICES];
tMPoly* mpoly;
tSawtooth* osc[NUM_VOICES];
tTalkbox* vocoder;

VocodecMode mode = FormantShiftMode;
AutotuneType atType = NearestType;

float formantShiftFactor;

/* PSHIFT vars *************/

int activeShifters = 1;
float pitchFactor;
float freq[NUM_VOICES];

float notePeriods[128];
int chordArray[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int lockArray[12];

/**********************************************/


HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;


void noteOn(int key, int velocity)
{
	int voice;
	if (!velocity)
	{
		//myVol = 0.0f;

		if (chordArray[key%12] > 0) chordArray[key%12]--;

		voice = tMPoly_noteOff(mpoly, key);
		if (voice >= 0) tRampSetDest(ramp[voice], 0.0f);
		for (int i = 0; i < mpoly->numVoices; i++)
		{
			if (tMPoly_isOn(mpoly, i) == 1)
			{
				freq[i] = OOPS_midiToFrequency(tMPoly_getPitch(mpoly, i));
				tSawtoothSetFreq(osc[i], freq[i]);
			}
		}

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);    //LED
	}
	else
	{
		chordArray[key%12]++;
		tMPoly_noteOn(mpoly, key, velocity);
		for (int i = 0; i < mpoly->numVoices; i++)
		{
			if (tMPoly_isOn(mpoly, i) == 1)
			{
				tRampSetDest(ramp[i], (float)(tMPoly_getVelocity(mpoly, i) * INV_TWO_TO_7));
				freq[i] = OOPS_midiToFrequency(tMPoly_getPitch(mpoly, i));
				tSawtoothSetFreq(osc[i], freq[i]);
			}
		}

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);    //LED3
	}
}

void noteOff(int key, int velocity)
{
	myVol = 0.0f;
	int voice;

	if (chordArray[key%12] > 0) chordArray[key%12]--;

	voice = tMPoly_noteOff(mpoly, key);
	if (voice >= 0) tRampSetDest(ramp[voice], 0.0f);
	for (int i = 0; i < mpoly->numVoices; i++)
	{
		if (tMPoly_isOn(mpoly, i) == 1)
		{
			freq[i] = OOPS_midiToFrequency(tMPoly_getPitch(mpoly, i));
			tSawtoothSetFreq(osc[i], freq[i]);
		}
	}

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);    //LED3
}

void ctrlInput(int ctrl, int value)
{

}

float nearestPeriod(float period)
{
	float leastDifference = fabsf(period - notePeriods[0]);
	float difference;
	int index = -1;

	int* chord = chordArray;
	if (lock > 0) chord = lockArray;

	for(int i = 0; i < 128; i++)
	{
		if (chord[i%12] > 0)
		{
			difference = fabsf(period - notePeriods[i]);
			if(difference < leastDifference)
			{
				leastDifference = difference;
				index = i;
			}
		}
	}

	if (index == -1) return period;

	return notePeriods[index];
}

void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn, RNG_HandleTypeDef* hrand, uint16_t* myADCArray)
{ 
	// Initialize the audio library. OOPS.
	OOPSInit(SAMPLE_RATE, HALF_BUFFER_SIZE, &randomNumber);

	for (int i = 0; i < 128; i++)
	{
		notePeriods[i] = 1.0f / OOPS_midiToFrequency(i) * oops.sampleRate;
	}
	fs = tFormantShifterInit();
	fs2 = tFormantShifterInit();
	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

	HAL_Delay(100);

	adcVals = myADCArray;

	mpoly = tMPoly_init(activeShifters);
	tMPoly_setPitchGlideTime(mpoly, 100.0f);

	for (int i = 0; i < NUM_VOICES; i++)
	{
		osc[i] = tSawtoothInit();
		ramp[i] = tRampInit(10.0f, 1);
	}

	vocoder = tTalkboxInit();

	// set up the I2S driver to send audio data to the codec (and retrieve input as well)	
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);

	/* Initialize devices for pitch shifting */
	for (int i = 0; i < NUM_SHIFTERS; ++i)
	{
		ps[i] = tPitchShifter_init(inBuffer[i], outBuffer[i], 4096, 1024);
		tPitchShifter_setWindowSize(ps[i], 1024);
		tPitchShifter_setHopSize(ps[i], 256);
		tPitchShifter_setPitchFactor(ps[i], 1.0f);
	}
}

float tempVal = 0.0f;
uint16_t frameCounter = 0;
int numSamples = AUDIO_FRAME_SIZE;

void audioFrame(void)
{
	float input, output;

	if (mode == FormantShiftMode)
	{
		for (int cc=0; cc < numSamples; cc++)
		{
			input = (float) (audioInBuffer[buffer_offset+(cc*2)] * INV_TWO_TO_15 * 2);
			formantShiftFactor = adcVals[1] * INV_TWO_TO_16;
			audioOutBuffer[buffer_offset + (cc*2)] = (int16_t) (tFormantShifterTick(fs, input, formantShiftFactor) * TWO_TO_15);
		}
	}
	else if (mode == PitchShiftMode)
	{
		for (int cc=0; cc < numSamples; cc++)
		{
			input = (float) (audioInBuffer[buffer_offset+(cc*2)] * INV_TWO_TO_15);

			pitchFactor = (adcVals[1] * INV_TWO_TO_16) * 3.5f + 0.5f;
			tPitchShifter_setPitchFactor(ps[0], pitchFactor);

			audioOutBuffer[buffer_offset + (cc*2)] = (int16_t) (tPitchShifter_tick(ps[0], input) * TWO_TO_15);
		}
	}
	else if (mode == AutotuneMode)
	{
		if (atType == NearestType)
		{
			for (int cc=0; cc < numSamples; cc++)
			{
				input = (float) (audioInBuffer[buffer_offset+(cc*2)] * INV_TWO_TO_15);

				audioOutBuffer[buffer_offset + (cc*2)] = (int16_t) (tPitchShifterToFunc_tick(ps[0], input, nearestPeriod) * TWO_TO_15);
			}
		}
		else if (atType == AbsoluteType)
		{
			for (int cc=0; cc < numSamples; cc++)
			{
				tMPoly_tick(mpoly);

				input = (float) (audioInBuffer[buffer_offset+(cc*2)] * INV_TWO_TO_15);
				output = 0.0f;

				for (int i = 0; i < activeShifters; ++i)
				{
					freq[i] = OOPS_midiToFrequency(tMPoly_getPitch(mpoly, i));
					output += tPitchShifterToFreq_tick(ps[i], input, freq[i]) * tRampTick(ramp[i]);
				}

				audioOutBuffer[buffer_offset + (cc*2)] = (int16_t) (output * TWO_TO_15);
			}
		}
	}
	if (mode == VocoderMode)
	{
		for (int cc=0; cc < numSamples; cc++)
		{
			tMPoly_tick(mpoly);

			input = (float) (audioInBuffer[buffer_offset+(cc*2)] * INV_TWO_TO_15);
			output = 0.0f;

			for (int i = 0; i < NUM_VOICES; i++)
			{
				freq[i] = OOPS_midiToFrequency(tMPoly_getPitch(mpoly, i));
				tSawtoothSetFreq(osc[i], freq[i]);
				output += tSawtoothTick(osc[i]) * tRampTick(ramp[i]);
			}
			output *= 0.25f;

			output = tTalkboxTick(vocoder, output, input);
			output = OOPS_softClip(output, 0.98f);
			audioOutBuffer[buffer_offset + (cc*2)]  = (int16_t) (output * TWO_TO_15);
		}
	}
}

float currentFreq = 1.0f;

float rightInput = 0.0f;

float audioTickL(float audioIn) 
{
	sample = 0.0f;

	return sample;
}

float audioTickR(float audioIn) 
{
	rightInput = audioIn;
	//sample = audioIn;
	return audioIn;
}


void writeStringToLCD(char* string, int len, int line, int space)
{
	GFXfillRect(&theGFX, 0, 0, 128, 16, 0);
	for (int i = 0; i < len; ++i)
	{

	}
}

char* modeNames[4] =
{
	"F0RMANT   ", "PITCHSHIFT",
	"AUTOTUNE  ", "VOCODER   "
};

#define ASCII_NUM_OFFSET 48
static void writeModeToLCD(VocodecMode in)
{
	OLEDwriteLine(modeNames[in], 10, FirstLine);
	if (in == AutotuneMode)
	{
		if ((atType == NearestType) && (lock > 0))
		{
			OLEDwriteLine("LOCK", 4, SecondLine);
		}
		else if (atType == AbsoluteType)
		{
			OLEDwriteIntLine(activeShifters, 2, SecondLine);
		}
		else OLEDwriteLine("          ", 10, SecondLine);
	}
	else OLEDwriteLine("          ", 10, SecondLine);
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
		atType = (atType == NearestType) ? AbsoluteType : NearestType;
	}
	else if (button == ButtonB)
	{
		if (mode == AutotuneMode)
		{
			if (atType == NearestType)
			{
				int notesHeld = 0;
				for (int i = 0; i < 12; ++i)
				{
					if (chordArray[i] > 0) { notesHeld = 1; }
				}

				if (lock > 0)
				{
					lock = 0;
				}
				else
				{
					if (notesHeld)
					{
						for (int i = 0; i < 12; ++i)
						{
							lockArray[i] = chordArray[i];
						}
					}

					lock = 1;
				}
			}
			else if (atType == AbsoluteType)
			{
				if (activeShifters < NUM_SHIFTERS) activeShifters++;
				else activeShifters = 1;
				mpoly->numVoices = activeShifters;
			}
		}
	}

	mode = (VocodecMode) modex;

	if (mode == AutotuneMode) tMPoly_setNumVoices(mpoly, activeShifters);
	if (mode == VocoderMode) tMPoly_setNumVoices(mpoly, NUM_VOICES);

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

