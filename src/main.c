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
 **          Author : uszko lukasz
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

/////////////////////////////////////////////////////////////////////
// declarations of functions
/////////////////////////////////////////////////////////////////////

void showWhereIam(uint8_t numberMenuRow, bool onORoff);
void showWhereIamSettings(uint8_t numberMenuRow, bool onORoff);
void setLCDBrightness(void);
void setSleepTime(void);
void calibrationMode(void);     // bigger functions used in main_menu
double setCalibrateFactor(int measuredCurrent);
void InternalADCSetup(void);
void drawBattery(uint32_t batVoltage);
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

//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////FFT FFT/////////////////////////////////////////////////////////////////////////////////////
#define BUFFER_SAMPLES  512  // number of samples for FFT
#define SAMPLE_RATE     1024   // sample rate used for sampling data.
float32_t floatBuf[BUFFER_SAMPLES];
float32_t fftOutputComplex[BUFFER_SAMPLES * 2]; // Complex output from FFT
float32_t fftOutputMag[BUFFER_SAMPLES]; //Magnitude of complex numbers in FFT output
volatile bool isFFTComputed = false; // Flag used to indicate whether fft is calculated
arm_rfft_instance_f32 rfft_instance; // Instance structures for float32_t real time fourier transform
arm_cfft_radix4_instance_f32 cfft_instance; // Instance structure for float32_t CFFT used by the RFFT

///////////////////////////////////////////////////////////////////////////
//                  bool doFFT(uint16_t x)                               //
//																		 //
//			  Process the sampled data through FFT.                      //
///////////////////////////////////////////////////////////////////////////
static void doFFT(uint16_t x) {
	static int i;
	floatBuf[i] = ((float32_t) ConvertU16_from_ADCToINT(x)) *  ADC_COEFFICIENT;
	i++;
	if (i == BUFFER_SAMPLES) {
		arm_rfft_f32(&rfft_instance, floatBuf, fftOutputComplex);
		arm_cmplx_mag_f32(fftOutputComplex, fftOutputMag, BUFFER_SAMPLES); // compute the magnitude of all complex results
		i = 0;
	   isFFTComputed=true;
	}

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//               float32_t GetMaxFreqFromFFT(void)                                                                   //
// 																													 //
//         																							                 //
// find a maximal peak in fft and returns its value its estimated with sinc interpolation'                           //
//  more details about this interpolation here:                                                                      //
//
/* Perform sinc() interpolation using the two bins on each side of the                                               //
 * maximal bin. For more information see page 113 of                                                                 //
 * http://tmo.jpl.nasa.gov/progress_report/42-118/118I.pdf                                                           //
 *///
// implementation of the algorithm from  :   http://cdn.energymicro.com/dl/an/pdf/an0051_efm32_dsp.pdf               //
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static float32_t GetMaxFreqFromFFT(void) {
	float32_t maxVal;
	uint32_t maxIndex;

	/* Real and imag components of maximal bin and bins on each side */
	float32_t rz_p, iz_p, rz_n, iz_n, rz_0, iz_0;
	/* Small correction to the "index" of the maximal bin */
	float32_t deltaIndex;
	/* Real and imag components of the intermediate result */
	float32_t a, b, c, d;

#define START_INDEX 4
	/*
	 * Find the biggest bin, disregarding the first bins because of DC offset and
	 * low frequency noise.
	 */
	arm_max_f32(&fftOutputMag[START_INDEX], BUFFER_SAMPLES / 2 - START_INDEX,
			&maxVal, &maxIndex);
	maxIndex += START_INDEX;
	rz_0 = fftOutputComplex[maxIndex * 2];
	iz_0 = fftOutputComplex[maxIndex * 2 + 1];
	rz_p = fftOutputComplex[maxIndex * 2 + 2];
	iz_p = fftOutputComplex[maxIndex * 2 + 2 + 1];
	rz_n = fftOutputComplex[maxIndex * 2 - 2];
	iz_n = fftOutputComplex[maxIndex * 2 - 2 + 1];
	a = rz_p - rz_n;
	b = iz_p - iz_n;
	c = rz_p + rz_n - (float32_t) 2.0 * rz_0;
	d = iz_p + iz_n - (float32_t) 2.0 * iz_0;
	deltaIndex = (a * c + b * d) / (c * c + d * d);
	return ((float32_t) maxIndex + deltaIndex) * (float32_t) SAMPLE_RATE
			/ (float32_t) BUFFER_SAMPLES;
	//return maxVal;
}

/////////////////////////////////////////////////////////////////////////////////
//           float32_t GetMagForGivenFrequncy(uint16_t binFrequency)           //
//																		       //
//			  returns value of magnitude for the given bin.                    //
/////////////////////////////////////////////////////////////////////////////////
static float32_t GetMagForGivenFrequncy(uint16_t binFrequency){

	uint16_t usefulFrequencies= BUFFER_SAMPLES/2;
	uint16_t resolution = SAMPLE_RATE/BUFFER_SAMPLES;
	if((binFrequency/resolution)>usefulFrequencies) return 0;
	return fftOutputMag[binFrequency/resolution+1];
}

//////////////////////////////////////////////////////////////////FFT FFT -- END/////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////FFT FFT -- END/////////////////////////////////////////////////////////////////////////////////////

Results results;
uint16_t FRAME = 0;
bool endOfADCInterrupt = false;
CircularBufferADC_Result Copy_ADC_RESULT;
char messageFromBTM222[4];
bool sendViaBTM222 = false;
bool isFFTEnable = false;
volatile bool messageFromBTMAvailable;
int main(void) {

	arm_status status; // status of fft
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
			init1msSystick();
			GLCD_Init();
			GLCD_ClearScreen();
			GLCD_GoTo(40, 3);
			GLCD_WriteString("LOADING...");
			GLCD_Enable();
			Delay(700);
			State.MODE = MAIN_MENU;
			GLCD_Enable();
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
				State.init = true;
			}
			showWhereIam(State.MAIN_MENU_CURRENT_OPTION, State.rtcFlag);

			break;
		}

		case MAIN_MENU_OPTION: {
			if (!State.init && State.LAST_MODE == SLEEP_MODE) {
				GLCD_Enable();
			}  //wake up
			switch (State.MAIN_MENU_CURRENT_OPTION) {

			case START: {
				if (!State.init) {
					GLCD_ClearScreen();
					GLCD_bmp(measure);

					status = arm_rfft_init_f32(&rfft_instance, &cfft_instance, // init fft functions
							BUFFER_SAMPLES, 0, 1);
					if (status != ARM_MATH_SUCCESS) { // Error initializing RFFT module.

						while (1)
							;
						isFFTComputed = false;
					}
					Set20MHzFrequency_Init();
					SPI2_setupRXInt_SW(&FRAME);
					Delay(10);
					BT_Enable();
					BTM222_Init();
					Timer1forDisplayResults_Setup();
					State.init = true;
					State.activeFunction.isMeasurementOn = true;
					sendViaBTM222 = true; ////////////////////////////////////////
					isFFTEnable=true;/////////////////////////////////////////////////////////////////////////////TODO
				}

				if (messageFromBTMAvailable) {
					BTM222_ReadData(messageFromBTM222);
					if (messageFromBTM222[3] == 'x') {
						switch (messageFromBTM222[0]) {
						case 'A':
							sendViaBTM222 = true;
							break; // start measure --> turn on sending data through bluetooth
						case 'B':
							sendViaBTM222 = false;
							break;
						case 'F':
							isFFTEnable = true;
							sendViaBTM222 = false;
							// stop sending other results
							break;
						case 'D':
							break;
						default:
							break;
						}
					}
					messageFromBTMAvailable = false;
				}

				if (endOfADCInterrupt) {
					endOfADCInterrupt = false;

					ResultADC_Buf_Write(&ADC_RESULT, FRAME);
			if (/*isFFTEnable&&*/(!isFFTComputed)) {
					     doFFT(FRAME); // fft
				}
					if (ADC_RESULT.Buf_isFull) {
						memcpy(&Copy_ADC_RESULT, &ADC_RESULT,
								sizeof(ADC_RESULT));
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
						GLCD_WriteString("SLAVE MODE");
						GLCD_GoTo(15, 5);
						GLCD_WriteString("HAS BEEN ENABLED");
						Delay(1000);
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
				GLCD_WriteString("00:12:6F:32:B4:F2");
				break;
			}
			case BAT_LEVEL: {
				if (!State.init) {
					GLCD_ClearScreen();
					GLCD_bmp(battery);
					InternalADCSetup();
					State.init = true;
					State.activeFunction.isBatteryLevelVerificated = true;
				}
				GLCD_GoTo(40, 3);
				GLCD_WriteString(" NO");
				BatVoltage batVoltage;
				batVoltage.batVoltageConstant = (results.batLevel + 800) / 1000;
				batVoltage.batVoltageFraction = (results.batLevel + 800) % 1000
						/ 10;
				snprintf(batVoltage.batVolatgeString, 5, "%d.%d",
						(int) batVoltage.batVoltageConstant,
						(int) batVoltage.batVoltageFraction);   // TODO
				GLCD_GoTo(40, 5);
				GLCD_WriteString(batVoltage.batVolatgeString);
				GLCD_GoTo(65, 5);
				GLCD_WriteString(" V");
				drawBattery(results.batLevel + 800);
				break;
			}
			case SD_CARD: {
				//not available yet
				GLCD_ClearScreen();
				GLCD_GoTo(40, 3);
				GLCD_WriteString("NO SD-CARD");
				GLCD_GoTo(20, 5);
				GLCD_WriteString("IN THE SOCKET !");
				Delay(1000);
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
				State.init = true;   // nie wraca
			}
			break;
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
//      void setLCDBrightness(void)                            //
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
//       void setSleepTime(void)                               //
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
//      void calibrationMode(void)                             //
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

			if (!GPIO_PinInGet(gpioPortE, LEFT)) {
						goto RETURN;
					}

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
	GLCD_ClearScreen();
	GLCD_bmp(callibration_over);
	gcvt(newFactor, 3, StringOutput);   //debug
		GLCD_GoTo(65, 5);   //debug
		GLCD_WriteString(StringOutput);   //debug
		Delay(8000);   //debug
	Delay(2000);
	RETURN:

	NVIC_EnableIRQ(GPIO_EVEN_IRQn); //turn off all external events interrupts
	NVIC_EnableIRQ(GPIO_ODD_IRQn);
	GLCD_ClearScreen();
	return;

}
/////////////////////////////////////////////////////////////////
//      double setCalibrateFactor(void)                        //
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

///////////////////////////////////////////////////////////////////////////
// Interrupt Service Routine TIMER1 Interrupt - displaying results       //
///////////////////////////////////////////////////////////////////////////
void TIMER1_IRQHandler(void) {
	char bufoo[8]; // sending buffer
	static uint8_t avgCount, dispCounter;

	results.rmsAVG[avgCount] = rms(&Copy_ADC_RESULT);
	results.maxAVG[avgCount] = max(&Copy_ADC_RESULT);
	results.minAVG[avgCount] = min(&Copy_ADC_RESULT);
	results.avgAVG[avgCount] = avg(&Copy_ADC_RESULT);
	avgCount++;
	dispCounter++;
	if (avgCount >= NUMBER_OF_VALUES_FOR_AVG) {
		avgCount = 0;
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
	}

	if (!(dispCounter % NUMBER_OF_VALUES_FOR_AVG)) {
		ConvertDOUBLEtoLCD(results.rms, bufoo, true);
		GLCD_GoTo(43, 2);
		GLCD_WriteString(bufoo);
		if (sendViaBTM222)
			BTM222_SendData(ParseDataToSendThroughBTM(bufoo, 'r', -1));
	} else if (!(dispCounter % (NUMBER_OF_VALUES_FOR_AVG + 1))) {
		ConvertDOUBLEtoLCD(results.max, bufoo, true);
		GLCD_GoTo(43, 4);
		GLCD_WriteString(bufoo);
		if (sendViaBTM222)
			BTM222_SendData(ParseDataToSendThroughBTM(bufoo, 'm', -1));
	} else if (!(dispCounter % (NUMBER_OF_VALUES_FOR_AVG + 2))) {
		ConvertDOUBLEtoLCD(results.min, bufoo, true);
		GLCD_GoTo(43, 3);
		GLCD_WriteString(bufoo);
		if (sendViaBTM222)
			BTM222_SendData(ParseDataToSendThroughBTM(bufoo, 'n', -1));
	} else if (!(dispCounter % (NUMBER_OF_VALUES_FOR_AVG + 3))) {
		ConvertDOUBLEtoLCD(results.avg, bufoo, true);
		GLCD_GoTo(43, 5);
		GLCD_WriteString(bufoo);
		if (sendViaBTM222)
			BTM222_SendData(ParseDataToSendThroughBTM(bufoo, 'a', -1));
	} else if (!(dispCounter % (NUMBER_OF_VALUES_FOR_AVG + 4))) {
		if (isFFTEnable && isFFTComputed) {
			static uint8_t binNumber;
			ConvertDOUBLEtoLCD(/*GetMaxFreqFromFFT()*/GetMagForGivenFrequncy(50*(binNumber+1)), bufoo, false);
			GLCD_GoTo(43, 7);
			GLCD_WriteString(bufoo);
			BTM222_SendData(ParseDataToSendThroughBTM(bufoo, 'f',binNumber));
			binNumber++;
			if(binNumber>9)binNumber=0;
			isFFTComputed = false;
		}
		double goertzel = doGoertzelAlgorithm(&Copy_ADC_RESULT); // for tests
		ConvertDOUBLEtoLCD(goertzel, bufoo, true);
		GLCD_GoTo(43, 6);
		GLCD_WriteString(bufoo);
		dispCounter = 0;
	}
	TIMER_IntClear(TIMER1, TIMER_IF_OF);
	for (volatile uint8_t i=0; i < 8; i++) {
		bufoo[i] = ' ';
	}
}

///////////////////////////////////////////////////////////////////////////
//           ADC0 Configuration for battery level                        //
///////////////////////////////////////////////////////////////////////////
void InternalADCSetup(void) {
	CMU_ClockEnable(cmuClock_ADC0, true);
	CMU_ClockEnable(cmuClock_PRS, true);
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef singleInit = ADC_INITSINGLE_DEFAULT;
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(4000000, 0);
	//init.ovsRateSel = ADC_CTRL_OVSRSEL_X4;
	ADC_Init(ADC0, &init);

	/* Init for single conversion use. */
	singleInit.reference = adcRefVDD;    //3.3V
	singleInit.input = adcSingleInpCh5;
	singleInit.resolution = adcRes12Bit;
	singleInit.prsEnable = true; // Enable PRS for ADC
	singleInit.prsSel = 0; // signal PRS from channel 0

	Timer2forInternalADCSampling_Setup();
	PRS_SourceSignalSet(0, PRS_CH_CTRL_SOURCESEL_TIMER2,
			PRS_CH_CTRL_SIGSEL_TIMER2OF, prsEdgePos); // Timer2 as source, TIMER2OF as signal (rising edge)//

	ADC_InitSingle(ADC0, &singleInit);
	ADC0 ->IEN = ADC_IEN_SINGLE; // Enable ADC Interrupt when Single Conversion Complete
	NVIC_SetPriority(ADC0_IRQn, 12);
	NVIC_EnableIRQ(ADC0_IRQn);
}

///////////////////////////////////////////////////////////////////////////
//           ADC0_IRQHandler                          					 //
//           Interrupt Service Routine for ADC                           //
///////////////////////////////////////////////////////////////////////////
void ADC0_IRQHandler(void) {
	ADC0 ->IFC = 1; // clear ADC0 flag
	/*   divider is made of 2 resistors that are connected in following way:   Vcc(3.3V)--->|3.3k|--|ucChannel5ADC|--|6.8k|---GND
	 *   then a coefficient of the divider is 0.67
	 *   reference Voltage : 3.3V -> resolution 0.0008V
	 *    Thus max measured voltage will be 2.747 [V]  , this situation may take place when the battery voltage is 4.1V
	 *    For 2.747 V (input) -> binary value from ADC : -> 3435
	 *
	 *    */
	results.batLevel = ADC_DataSingleGet(ADC0 );
	results.batLevel = ((((results.batLevel * 3300) / 4096)) / 0.67);

}

///////////////////////////////////////////////////////////////////////////
// function that draws the battery graphic                                 //
///////////////////////////////////////////////////////////////////////////
void drawBattery(uint32_t batVoltage) {
	///////////////////// 26- 43 pixels ///////////////////////
	///////////////////// 87- 126 pixels //////////////////////
	// 40 bat levels levels
	uint8_t batLevel = 0; // if the value is higher , batVoltage is smaller;
	if (batVoltage > 4100)
		batLevel = 2;
	else if (batVoltage < 4100 && batVoltage >= 3900)
		batLevel = 4;
	else if (batVoltage < 3900 && batVoltage >= 3700)
		batLevel = 8;
	else if (batVoltage < 3700 && batVoltage >= 3500)
		batLevel = 16;
	else if (batVoltage < 3500 && batVoltage >= 3300)
		batLevel = 24;
	else if (batVoltage < 3300 && batVoltage >= 3100)
		batLevel = 32;
	else
		batLevel = 40;
	for (uint8_t x = 87 + batLevel; x < 127; x++) {
		for (uint8_t y = 26; y < 44; y++) {
			GLCD_SetPixel(x, y, true);
		}
	}
}

