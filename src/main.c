/**
 *****************************************************************************
 **
 **  File        : main.c
 **
 **  Abstract    : main function.
 **
 **  Functions   : main
 **
 **  Environment : Atollic TrueSTUDIO
 **
 **
 **          Author : uszko, kwiecinski
 **
 *****************************************************************************
 */

/* Includes */
#include <stdint.h>
#include <stdbool.h>
#include "efm32.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "splc501c.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_adc.h"
#include "em_prs.h"
#include "em_timer.h"
#include "em_lcd.h"
#include "em_usart.h"
#include "em_rtc.h"
#include "ADS7835E.h"
#include "Timers.h"
#include "BTM222.h"
#include "goertzel.h"
#include "keyboard.h"
#include "bitmaps.h"
#include "arm_math.h"

///////////////////////////////////////////// DEFINE'y KOMPILACJI WARUNKOWEJ//////////////////////////////////////////////
#define DEBUG 1           // dla debugerra- for testing
#define COMPILATION 1   // 1 sprzetowa obsl USART, 0 programowa
#define SAMPLING_TIMER0 1 // jesli probkuje w przerwaniu od timera to 1
#define	SIGN_TEST 0

void showWhereIam(uint8_t numberMenuRow, bool onORoff);
void showWhereIamSettings(uint8_t numberMenuRow, bool onORoff);

void setLCDBrightness(void);
void setSleepTime(void);
void calibrationMode(void);     // bigger functions used in main_menu
double setCalibrateFactor(int measuredCurrent);
/////////////////////////////////////////////////////////////////////
//functions that allow you to TURN ON/OFF the following module
/////////////////////////////////////////////////////////////////////
static void GLCD_Enable(void) {
	GPIO_PinModeSet(gpioPortA, 13, gpioModePushPull, 0);
	GPIO_PinOutSet(gpioPortA, 13);
}

static void GLCD_Disable(void) {
	GPIO_PinModeSet(gpioPortA, 13, gpioModePushPull, 0);
	GPIO_PinOutClear(gpioPortA, 13);
}
static void BT_Enable(void) {
	GPIO_PinModeSet(gpioPortF, 6, gpioModePushPull, 0);
	GPIO_PinOutClear(gpioPortF, 6);
}

static void BT_Disable(void) {
	GPIO_PinModeSet(gpioPortF, 6, gpioModePushPull, 0);
	GPIO_PinOutSet(gpioPortF, 6);
}

static void SDCard_Enable(void) {

	GPIO_PinModeSet(gpioPortF, 12, gpioModePushPull, 1);
	GPIO_PinOutClear(gpioPortF, 12);
}

static void SDCard_Disable(void) {
	GPIO_PinModeSet(gpioPortF, 12, gpioModePushPull, 1);
	GPIO_PinOutSet(gpioPortF, 12);
}

/////////////////////////////////////////////////////////////////////

static void Set20MHzFrequency_Init(void) {
	/* Use crystal oscillator for HFXO */
	CMU ->CTRL |= CMU_CTRL_HFXOMODE_XTAL;
	/* HFXO setup */CMU ->CTRL = (CMU ->CTRL & ~_CMU_CTRL_HFXOBOOST_MASK)
			| CMU_CTRL_HFXOBOOST_50PCENT;

	/* Enable HFXO as high frequency clock, HFCLK */
	CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);
	CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);

}
////////////////////////////////////////////////////////////////////////////////

const states init_States = STATES_INIT_DEFAULT; //default settings
volatile states State;
///////////////////////////////////////////////////////////////////////////////////////////////////
void Delay(uint32_t dlyTicks);
volatile uint32_t msTicks; /* counts 1ms timeTicks */

///////////////////GLOBAL VARIABLES////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////

/** Buffer of float samples ready for FFT. */
#define BUFFER_SAMPLES                  512

/** (Approximate) sample rate used for sampling data. */
#define SAMPLE_RATE                     (1024)
static float32_t floatBuf[BUFFER_SAMPLES];

/** Complex (interleaved) output from FFT. */
static float32_t fftOutputComplex[BUFFER_SAMPLES * 2];

/** Magnitude of complex numbers in FFT output. */
static float32_t fftOutputMag[BUFFER_SAMPLES];

/** Flag used to indicate whether data is ready for processing */
static volatile bool dataReadyForFFT;
/** Indicate whether we are currently processing data through FFT */
static volatile bool processingFFT;

/** Instance structures for float32_t RFFT */
static arm_rfft_instance_f32 rfft_instance;
/** Instance structure for float32_t CFFT used by the RFFT */
static arm_cfft_radix4_instance_f32 cfft_instance;

/***************************************************************************//**
 * @brief
 *   Process the sampled data through FFT.
 *******************************************************************************/bool ProcessFFT(
		uint16_t x) {
	static int i;

	/*
	 * Convert to float values.
	 */

	floatBuf[i] = ((float32_t) ConvertU16_from_ADCToINT(x)) * 0.00122;
	i++;

	/* Process the data through the RFFT module, resulting complex output is
	 * stored in fftOutputComplex
	 */
	if (i == BUFFER_SAMPLES) {
		arm_rfft_f32(&rfft_instance, floatBuf, fftOutputComplex);

		/* Compute the magnitude of all the resulting complex numbers */
		arm_cmplx_mag_f32(fftOutputComplex, fftOutputMag, BUFFER_SAMPLES);
		i = 0;
		return true;
	}
	return false;
}

/***************************************************************************//**
 * @brief
 *   Find the maximal bin and estimate the frequency using sinc interpolation.
 * @return
 *   Frequency of maximal peak
 *******************************************************************************/
float32_t GetFreq(void) {
	float32_t maxVal;
	uint32_t maxIndex;

	/* Real and imag components of maximal bin and bins on each side */
	float32_t rz_p, iz_p, rz_n, iz_n, rz_0, iz_0;
	/* Small correction to the "index" of the maximal bin */
	float32_t deltaIndex;
	/* Real and imag components of the intermediate result */
	float32_t a, b, c, d;

#define START_INDEX 4
	/* Find the biggest bin, disregarding the first bins because of DC offset and
	 * low frequency noise.
	 */
	arm_max_f32(&fftOutputMag[START_INDEX], BUFFER_SAMPLES / 2 - START_INDEX,
			&maxVal, &maxIndex);

	maxIndex += START_INDEX;

	/* Perform sinc() interpolation using the two bins on each side of the
	 * maximal bin. For more information see page 113 of
	 * http://tmo.jpl.nasa.gov/progress_report/42-118/118I.pdf
	 */

	/* z_{peak} */
	rz_0 = fftOutputComplex[maxIndex * 2];
	iz_0 = fftOutputComplex[maxIndex * 2 + 1];

	/* z_{peak+1} */
	rz_p = fftOutputComplex[maxIndex * 2 + 2];
	iz_p = fftOutputComplex[maxIndex * 2 + 2 + 1];

	/* z_{peak-1} */
	rz_n = fftOutputComplex[maxIndex * 2 - 2];
	iz_n = fftOutputComplex[maxIndex * 2 - 2 + 1];

	/* z_{peak+1} - z_{peak-1} */
	a = rz_p - rz_n;
	b = iz_p - iz_n;
	/* z_{peak+1} + z_{peak-1} - 2*z_{peak} */
	c = rz_p + rz_n - (float32_t) 2.0 * rz_0;
	d = iz_p + iz_n - (float32_t) 2.0 * iz_0;

	/* Re (z_{peak+1} - z_{peak-1}) / (z_{peak+1} + z_{peak-1} - 2*z_{peak}) */
	deltaIndex = (a * c + b * d) / (c * c + d * d);

	return ((float32_t) maxIndex + deltaIndex) * (float32_t) SAMPLE_RATE
			/ (float32_t) BUFFER_SAMPLES;

	//return  maxVal;
}

//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
char receivexBuffer[10];
Results results;
bool endOfADCInterrupt;
uint16_t FRAME = 0;
bool endOfADCInterrupt = false;
int licznik_kwiecinskigo = 0;
uint8_t avgCounter = 0;

//char buf[10];
/////////////////////////13//////////////////////////////////////////////////////////////////////////////
// value of rms
int main(void) {

	uint32_t time;
	arm_status status; //////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////

	CHIP_Init();
	CircularBufferADC_Result ADC_RESULT;
	ResultADC_Buf_Init(&ADC_RESULT, SIZE_BUF_ADC); // 64 samples /sampling freuency is 1000Hz
	KeyboardGpioSetup(); // init keyboard , external events
	State = init_States;
	while (1) {
		switch (State.MODE) {
		case WAITFORENABLE: {
			EMU_EnterEM2(true);   // deep sleep mode
			break;
		}
		case ENABLING: {
			/* Setup SysTick Timer for 1 msec interrupts  */
			if (SysTick_Config(CMU_ClockFreqGet(cmuClock_CORE) / 1000))
				while (1)
					;
			GLCD_Init();
			GLCD_ClearScreen();
			GLCD_GoTo(40, 3);
			GLCD_WriteString("LOADING...");
			GLCD_Enable();
			Delay(700);
			State.MODE = MAIN_MENU;
			GLCD_Enable();
			char respBuf1[10];

			break;
		}

		case MAIN_MENU: {
			if (!State.init && State.LAST_MODE == SLEEP_MODE) {
				GLCD_Enable();
			}  //wake up
			if (!State.init) {
				GLCD_ClearScreen();
				GLCD_bmp(main_menu);
				GLCD_GoTo(1, 1);
				/*    licznik_kwiecinskigo++;
				 snprintf(buf, 10, "NR: %d", licznik_kwiecinskigo);
				 GLCD_WriteString(buf);
				 */
				State.init = true;
			}
			showWhereIam(State.MAIN_MENU_CURRENT_OPTION, State.rtcFlag);

			break;
		}

		case MAIN_MENU_OPTION: {

			//GLCD_GoTo(1, 1);
			/*    licznik_kwiecinskigo++;
			 snprintf(buf, 10, "NR: %d", licznik_kwiecinskigo);
			 GLCD_WriteString(buf);
			 */

			if (!State.init && State.LAST_MODE == SLEEP_MODE) {
				GLCD_Enable();
			}  //wake up
			switch (State.MAIN_MENU_CURRENT_OPTION) {

			case START: {
				if (!State.init) {
					GLCD_ClearScreen();
					GLCD_bmp(measure);
					/* Initialize the CFFT/CIFFT module */
					status = arm_rfft_init_f32(&rfft_instance, &cfft_instance,
							BUFFER_SAMPLES, 0, /* forward transform */1); /* normal, not bitreversed, order */
					if (status != ARM_MATH_SUCCESS) {
						/* Error initializing RFFT module. */
						while (1)
							;
					}
					Set20MHzFrequency_Init();
					//cmuSetup();
					SPI2_setupRXInt_SW(&FRAME);
					Delay(10);
					BT_Enable();
					BTM222_Init();
					State.init = true;
					State.activeFunction.isMeasurementOn = true;
				}
				static uint16_t BTMcounter;
				char bufoo[7]; // receive buffer
				//BTM222_ReadData(respBuf1);
				//while(1){
				if (endOfADCInterrupt) {
					/* Do FFT, measure number of cpu cycles used. */
					/*
					 if (ProcessFFT(FRAME)) {
					 ConvertDOUBLEtoLCD(GetFreq(), bufoo, false);
					 GLCD_GoTo(50, 2);
					 GLCD_WriteString(bufoo);
					 }


					 */
					/*
					 ConvertDOUBLEtoLCD((double) time, bufoo);
					 GLCD_GoTo(50, 3);
					 GLCD_WriteString(bufoo);
					 */
					endOfADCInterrupt = false;

					ResultADC_Buf_Write(&ADC_RESULT, FRAME);
					if (ADC_RESULT.Buf_isFull) {
						BTMcounter++;
						//	if (BTMcounter == 200) {
						/*
						 results.avg = avg(&ADC_RESULT);
						 results.max = max(&ADC_RESULT);
						 results.min = min(&ADC_RESULT);
						 ConvertDOUBLEtoLCD(results.max, bufoo, true);
						 GLCD_GoTo(50, 4);
						 GLCD_WriteString(bufoo);
						 BTM222_SendData(ParseDataToSendThroughBTM(bufoo, 'm'));
						 ConvertDOUBLEtoLCD(results.min, bufoo, true);
						 GLCD_GoTo(50, 3);
						 GLCD_WriteString(bufoo);
						 BTM222_SendData(ParseDataToSendThroughBTM(bufoo, 'n'));
						 ConvertDOUBLEtoLCD(results.avg, bufoo, true);
						 GLCD_GoTo(50, 5);
						 GLCD_WriteString(bufoo);
						 BTM222_SendData(ParseDataToSendThroughBTM(bufoo, 'a'));
						 BTMcounter = 0;

						 */
						/*
						 results.rms = rms(&ADC_RESULT);
						 ConvertDOUBLEtoLCD(results.rms, bufoo);
						 GLCD_GoTo(50, 2);
						 GLCD_WriteString(bufoo);
						 BTM222_SendData(ParseDataToSendThroughBTM(bufoo, 'r'));

						 */
						//		 }
						/*
						 results.rmsAVG[avgCounter] = rms(&ADC_RESULT);
						 results.maxAVG[avgCounter] = max(&ADC_RESULT);
						 results.minAVG[avgCounter] = min(&ADC_RESULT);
						 results.avgAVG[avgCounter] = avg(&ADC_RESULT);
						 avgCounter++;
						 if (avgCounter >= NUMBER_OF_VALUES_FOR_AVG) {
						 avgCounter = 0;
						 results.rms = 0;
						 results.min = 0;
						 results.max = 0;
						 results.avg = 0;
						 for (int i = 0; i < NUMBER_OF_VALUES_FOR_AVG; i++) {
						 results.rms += results.rmsAVG[i];
						 results.min += results.minAVG[i];
						 results.max += results.maxAVG[i];
						 results.avg += results.avgAVG[i];
						 }
						 results.rms /= NUMBER_OF_VALUES_FOR_AVG;
						 results.min /= NUMBER_OF_VALUES_FOR_AVG;
						 results.max /= NUMBER_OF_VALUES_FOR_AVG;
						 results.avg /= NUMBER_OF_VALUES_FOR_AVG;
						 //ConvertDOUBLEtoLCD(results.rms, bufoo, true);
						 //GLCD_GoTo(50, 2);
						 //GLCD_WriteString(bufoo);
						 ConvertDOUBLEtoLCD(results.max, bufoo, true);
						 GLCD_GoTo(50, 4);
						 GLCD_WriteString(bufoo);
						 ConvertDOUBLEtoLCD(results.min, bufoo, true);
						 GLCD_GoTo(50, 3);
						 GLCD_WriteString(bufoo);
						 ConvertDOUBLEtoLCD(results.avg, bufoo, true);
						 GLCD_GoTo(50, 5);
						 GLCD_WriteString(bufoo);
						 }
						 */
						/*
						 InitGoertzel();
						 double goertzel = doGoertzelAlgorithm(&ADC_RESULT); // for tests
						 ConvertDOUBLEtoLCD(goertzel, bufoo);
						 GLCD_GoTo(50, 6);
						 GLCD_WriteString(bufoo);
						 */
						if (BTMcounter == 8) {
							double goertzel = doGoertzelAlgorithm(&ADC_RESULT); // for tests
							ConvertDOUBLEtoLCD(goertzel, bufoo, true);
							GLCD_GoTo(50, 6);
							GLCD_WriteString(bufoo);
							BTMcounter = 0;
						}

							ADC_RESULT.Buf_isFull = false;
					}

				}

				break;
			}
			case SETTINGS: {
				if (!State.init) {
					GLCD_ClearScreen();
					GLCD_bmp(settings);
					State.init = true;
				}

				if (State.activeFunction.settings_menu) {
					switch (State.SETTINGS_CURRENT_OPTION) {
					case SLAVE_MODE:
						GLCD_ClearScreen();
						GLCD_GoTo(40, 3);
						GLCD_WriteString("SLAVE MODE.");
						break;
					case SET_TIME:
						break;
					case SLEEP_TIME:
						setSleepTime();
						State.activeFunction.settings_menu = false;
						State.init = false; //powrot re-draw again
						break;
					case LCD_BRIGHTNESS:
						setLCDBrightness();
						State.activeFunction.settings_menu = false;
						State.init = false; //powrot re-draw again
						break;
					default:
						break;
					}
				} else
					showWhereIamSettings(State.SETTINGS_CURRENT_OPTION,
							State.rtcFlag);
				break;
			}
			case BT_STATE: {
				if (!State.init) {
					GLCD_ClearScreen();
					GLCD_bmp(bt_state);
					State.init = true;
				}
				GLCD_GoTo(2, 3);
				GLCD_WriteString("CONNECTED");
				GLCD_GoTo(2, 5);
				GLCD_WriteString(" MAC ...");
				break;
			}
			case BAT_LEVEL: {
				if (!State.init) {
					GLCD_ClearScreen();
					GLCD_bmp(battery);
					State.init = true;
				}
				GLCD_GoTo(50, 3);
				GLCD_WriteString("YES");
				GLCD_GoTo(50, 5);
				GLCD_WriteString("91 %");
				break;
			}
			case SD_CARD: {
				break;
			}
			case CALIBRATION: {
				if (!State.init) {
					Set20MHzFrequency_Init();
					//ADD spi frame for ADC similar to START
					GLCD_ClearScreen();
					GLCD_bmp(callibration_start);
					State.init = true;
				}
				calibrationMode();
				State.init = false;   // return to main menu
				State.MODE = MAIN_MENU;

				break;
			}
			default: {
				break;
			}
			}
			EMU_EnterEM1();
			break;
		}
		case BATTERY_ALARM: {
			if (!State.init) {
				GLCD_ClearScreen();
				GLCD_bmp(battery_alarm);
				State.init = true;   //// nie wracaaaaaaaaaaaaaaaaaaaaaaaaa
				break;
			}
		}
		case SLEEP_MODE: {
			if (!State.init) {
				GLCD_ClearScreen();
				GLCD_Disable();
				BT_Disable();
				State.init = true;
			}
			EMU_EnterEM2(true);
			break;
		}

		}
	}
}

void SysTick_Handler(void) {
	msTicks++; /* increment counter necessary in Delay()*/
}

/////////////////////////////////////////////////////////
void Delay(uint32_t dlyTicks) {
	uint32_t currentTicks;
	currentTicks = msTicks;
	while ((msTicks - currentTicks) < dlyTicks)
		;
}
/////////////////////////////////////////////////////////
void showWhereIam(uint8_t numberMenuRow, bool onORoff) {
	if (numberMenuRow > NUMBER_OF_OPTIONS)
		return;
	static uint8_t lastNumberRow;
	if (lastNumberRow != numberMenuRow) {
		for (uint8_t x = 0; x < 17; x++) {
			for (uint8_t y = 0; y < 5; y++) {
				GLCD_SetPixel(x + 2, y + 17 + 8 * lastNumberRow, true);
				GLCD_SetPixel(x + 113, y + 17 + 8 * lastNumberRow, true);
			}
		}
	}
	for (uint8_t x = 0; x < 17; x++) {
		for (uint8_t y = 0; y < 5; y++) {
			GLCD_SetPixel(x + 2, y + 17 + 8 * numberMenuRow,
					onORoff ? true : false);
			GLCD_SetPixel(x + 113, y + 17 + 8 * numberMenuRow,
					onORoff ? true : false);
		}
	}
	lastNumberRow = numberMenuRow;
}
/////////////////////////////////////////////////////////////////
//   void showWhereIamSettings(uint8_t numberMenuRow, bool onORoff)
/////////////////////////////////////////////////////////////////
void showWhereIamSettings(uint8_t numberMenuRow, bool onORoff) {
	if (numberMenuRow > NUMBER_OF_SETTINGS_OPTIONS)
		return;
	static uint8_t lastNumberRow;
	if (lastNumberRow != numberMenuRow) {
		for (uint8_t x = 0; x < 17; x++) {
			for (uint8_t y = 0; y < 5; y++) {
				GLCD_SetPixel(x + 2, y + 17 + 8 * lastNumberRow, true);
				GLCD_SetPixel(x + 113, y + 17 + 8 * lastNumberRow, true);
			}
		}
	}
	for (uint8_t x = 0; x < 17; x++) {
		for (uint8_t y = 0; y < 5; y++) {
			GLCD_SetPixel(x + 2, y + 17 + 8 * numberMenuRow,
					onORoff ? true : false);
			GLCD_SetPixel(x + 113, y + 17 + 8 * numberMenuRow,
					onORoff ? true : false);
		}
	}
	lastNumberRow = numberMenuRow;
}
/////////////////////////////////////////////////////////////////
//      void setLCDBrightness(void)
/////////////////////////////////////////////////////////////////
void setLCDBrightness(void) {

	GPIO_IntConfig(gpioPortE, OK, false, true, false); // turn off during setting
	GPIO_IntConfig(gpioPortE, UP, false, true, false);
	GPIO_IntConfig(gpioPortE, DOWN, false, true, false);

	GLCD_ClearScreen();
	GLCD_bmp(lcd_brightness);
	uint8_t brightness = 20;
	uint8_t percentValue = (brightness * 100) / (64);
	char str[5];
	Delay(300);
	while (GPIO_PinInGet(gpioPortE, OK)) {
		GLCD_WriteCommand(SPLC501C_VOLUME_MODE);
		GLCD_WriteCommand(brightness);
		snprintf(str, 4, " %d ", percentValue);
		//str[4] = '%';
		GLCD_GoTo(60, 5);
		GLCD_WriteString(str);
		GLCD_WriteString(" %");
		if (!GPIO_PinInGet(gpioPortE, DOWN)) {
			brightness -= 4;
			brightness %= 64;
			Delay(500);
		}
		if (!GPIO_PinInGet(gpioPortE, UP)) {
			brightness += 4;
			brightness %= 64;
			Delay(500);
		}
		percentValue = (brightness * 100) / (64);
	}
	GPIO_IntConfig(gpioPortE, OK, false, true, true); // turn on back after setting
	GPIO_IntConfig(gpioPortE, UP, false, true, true);
	GPIO_IntConfig(gpioPortE, DOWN, false, true, true);
	Delay(300);
}

/////////////////////////////////////////////////////////////////
//       void setSleepTime(void)
/////////////////////////////////////////////////////////////////
void setSleepTime(void) {

	NVIC_DisableIRQ(GPIO_EVEN_IRQn); //turn off all external events interrupts
	NVIC_DisableIRQ(GPIO_ODD_IRQn);

	GLCD_ClearScreen();
	GLCD_bmp(sleepTime);
	uint16_t sleepTime = State.deepSleepTime;

	char StringOutput[5];
	Delay(300);
	while (GPIO_PinInGet(gpioPortE, OK)) {
		snprintf(StringOutput, 5, " %1d", sleepTime);
		GLCD_GoTo(58, 5);
		GLCD_WriteString(StringOutput);
		GLCD_GoTo(100, 5);
		GLCD_WriteString("[s]");

		if (!GPIO_PinInGet(gpioPortE, UP)) {

			Delay(60);
			if (!GPIO_PinInGet(gpioPortE, UP)) {

				sleepTime++;
				if (sleepTime >= 3600) {
					sleepTime = 3600;
				}
			}
		}
		if (!GPIO_PinInGet(gpioPortE, DOWN)) {
			Delay(60);
			if (!GPIO_PinInGet(gpioPortE, DOWN)) {
				sleepTime--;
				if (sleepTime <= 0) {
					sleepTime = 0;
				}
			}
		}
		if (!GPIO_PinInGet(gpioPortE, LEFT)) {
			goto RETURN;
		}

	}
	State.deepSleepTime = sleepTime;
	RETURN: NVIC_EnableIRQ(GPIO_EVEN_IRQn); //turn off all external events interrupts
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	Delay(300);
}

/////////////////////////////////////////////////////////////////
//      void calibrationMode(void)
/////////////////////////////////////////////////////////////////
void calibrationMode(void) {
	NVIC_DisableIRQ(GPIO_EVEN_IRQn); //turn off all external events interrupts
	NVIC_DisableIRQ(GPIO_ODD_IRQn);
	uint8_t numberOfCalibration = 2;
	uint8_t i = 0;
	double scallingFactors[2]; // tu trzymane przed obliczeniem ostatecznego
	double newFactor = 0; // tu trzymane przed obliczeniem ostatecznego
	int currentValue = 0;
	char StringOutput[5];
	Delay(500);
	while (GPIO_PinInGet(gpioPortE, OK) && GPIO_PinInGet(gpioPortE, RIGHT)) {

		if (!GPIO_PinInGet(gpioPortE, LEFT)) {
			goto RETURN;
		}
	}

	while (i < (numberOfCalibration)) {
		GLCD_ClearScreen();
		GLCD_bmp(callibration);
		currentValue = 0;
		i++;
		GLCD_GoTo(12, 5);
		if (i == 1) {

			GLCD_WriteString("1");
		}

		else
			GLCD_WriteString("2");

		Delay(400);
		while (GPIO_PinInGet(gpioPortE, RIGHT) && GPIO_PinInGet(gpioPortE, OK)) {
			snprintf(StringOutput, 5, " %3d", currentValue);
			GLCD_GoTo(65, 5);
			GLCD_WriteString(StringOutput);

			if (!GPIO_PinInGet(gpioPortE, UP)) {

				Delay(60);
				if (!GPIO_PinInGet(gpioPortE, UP)) {

					currentValue++;
					if (currentValue >= 500) {
						currentValue = 500;
					}
				}
			}
			if (!GPIO_PinInGet(gpioPortE, DOWN)) {
				Delay(60);
				if (!GPIO_PinInGet(gpioPortE, DOWN)) {
					currentValue--;
					if (currentValue <= 0) {
						currentValue = 0;
					}
				}
			}
		}
		GLCD_ClearScreen();
		GLCD_bmp(working);
		scallingFactors[i - 1] = setCalibrateFactor(currentValue);
		Delay(5000);
		GLCD_ClearScreen();
		GLCD_bmp(succes);
		Delay(3000);
	}
	for (uint8_t i = 0; i < 2; i++) {
		newFactor += scallingFactors[i];

	}

	newFactor = newFactor / 2;             //TODO
	RETURN: GLCD_ClearScreen();
	GLCD_bmp(callibration_over);
	gcvt(newFactor, 3, StringOutput);   //debug
	GLCD_GoTo(65, 5);   //debug
	GLCD_WriteString(StringOutput);   //debug
	Delay(8000);   //debug
	NVIC_EnableIRQ(GPIO_EVEN_IRQn); //turn off all external events interrupts
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	return;

}
/////////////////////////////////////////////////////////////////
//      double setCalibrateFactor(void)
/////////////////////////////////////////////////////////////////
double setCalibrateFactor(int measuredCurrent) {
	uint16_t measured_data; //posluzy do odbierania ramek od niej
	static int previousMeasuredCurrent;
	CircularBufferADC_Result ADC_RESULT;
	ResultADC_Buf_Init(&ADC_RESULT, SIZE_BUF_ADC); // 100 samples /sampling freuency is 100Hz
	SPI2_setupRXInt_SW(&measured_data); // setup
	const uint8_t numberOfSamples = SIZE_BUF_ADC;
	double tempBuf[numberOfSamples];
	double avgRms = 0;
	double factor = 0;

	uint8_t i = 0;
	while (i < numberOfSamples) {

		if (endOfADCInterrupt) {
			endOfADCInterrupt = false;
			ResultADC_Buf_Write(&ADC_RESULT, measured_data);
			if (ADC_RESULT.Buf_isFull) {
				tempBuf[i] = rms(&ADC_RESULT);
				i++;
			}
		}
	}
	SPI2_disableRXInt_SW();
	for (uint8_t i = 0; i < numberOfSamples; i++) {
		avgRms += tempBuf[i];
	}
	avgRms = avgRms / numberOfSamples;
	if (avgRms == 0 || measuredCurrent == 0)
		measuredCurrent = previousMeasuredCurrent;
	factor = measuredCurrent / avgRms;
	previousMeasuredCurrent = measuredCurrent;
	return factor;
}

