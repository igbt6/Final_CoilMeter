/*
 * goertzel.h
 *
 *  Created on: Dec 2, 2013
 *      Author: lukasz
 */

#ifndef GOERTZEL_H_
#define GOERTZEL_H_
#include <stdint.h>

#define FLOATING	float
#define SAMPLE	uint16_t

void InitGoertzel(void);
void ResetGoertzel(void);
/* Call this routine for every sample. */
void ProcessSample(SAMPLE sample);

/* Basic Goertzel */
/* Call this routine after every block to get the complex result. */
void GetRealImag(FLOATING *realPart, FLOATING *imagPart);

/* Optimized Goertzel */
/* Call this after every block to get the RELATIVE magnitude squared. */
FLOATING GetMagnitudeSquared(void);

float goertzel(int numSamples,int _FREQUENCY,int _RATE, uint16_t* data);
#endif /* GOERTZEL_H_ */


