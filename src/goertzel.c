/*
 * goertzel.c
 *
 *  Created on: Dec 2, 2013
 *      Author: lukasz
 */





#include <stdio.h>
#include "goertzel.h"
#include "ADS7835E.h"
#include "arm_math.h"

#define SAMPLING_RATE	1000.0	//8kHz
#define TARGET_FREQUENCY	50.0	//941 Hz
#define N	100	//Block size



FLOATING coeff;
FLOATING Q1;
FLOATING Q2;
FLOATING sine;
FLOATING cosine;



/* Call this routine before every "block" (size=N) of samples. */
void ResetGoertzel(void)
{
  Q2 = 0;
  Q1 = 0;
}
/* Call this once, to precompute the constants. */
void InitGoertzel(void)
{
  int	k;
  FLOATING	floatN;
  FLOATING	omega;

  floatN = (FLOATING) N;
  k = (int) (0.5 + ((floatN * TARGET_FREQUENCY) / SAMPLING_RATE));
  omega = (2.0 * PI * k) / floatN;
  sine = sin(omega);
  cosine = cos(omega);
  coeff = 2.0 * cosine;
  ResetGoertzel();
}

/* Call this routine for every sample. */
void ProcessSample(SAMPLE sample)
{
  FLOATING Q0;
  Q0 = coeff * Q1 - Q2 + (FLOATING) sample;
  Q2 = Q1;
  Q1 = Q0;
}


/* Basic Goertzel */
/* Call this routine after every block to get the complex result. */
void GetRealImag(FLOATING *realPart, FLOATING *imagPart)
{
  *realPart = (Q1 - Q2 * cosine)/((float)(N/2));  // scalling factory
  *imagPart = (Q2 * sine)/((float)(N/2));
}

/* Optimized Goertzel */
/* Call this after every block to get the RELATIVE magnitude squared. */
FLOATING GetMagnitudeSquared(void)
{
  FLOATING result;

  result = Q1 * Q1 + Q2 * Q2 - Q1 * Q2 * coeff;
  return result;
}






float goertzel_mag(int numSamples,int _FREQUENCY,int _RATE, uint16_t* data)

{
    int     k,i;
    float   floatnumSamples;
    float   omega,sine,cosine,coeff,q0,q1,q2,magnitude,real,imag;

    float   scalingFactor = numSamples / 2.0;

    floatnumSamples = (float) numSamples;
    k = (int) (0.5 + ((floatnumSamples * _FREQUENCY) / _RATE));
    omega = (2.0 * PI * k) / floatnumSamples;
    sine = sin(omega);
    cosine = cos(omega);
    coeff = 2.0 * cosine;
    q0=0;
    q1=0;
    q2=0;

    for(i=0; i<numSamples; i++)
    {
        q0 = coeff * q1 - q2 + (float)ConvertU16_from_ADCToINT(data[i]);
        q2 = q1;
        q1 = q0;
    }

    // calculate the real and imaginary results
    // scaling appropriately
    real = (q1 - q2 * cosine) / scalingFactor;
    imag = (q2 * sine) / scalingFactor;

    magnitude = sqrtf(real*real + imag*imag);
    return magnitude;
}
