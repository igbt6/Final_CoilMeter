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

float goertzel_mag(int numSamples,int _FREQUENCY,int _RATE, uint16_t* data);
#endif /* GOERTZEL_H_ */



//kod ze stacka

/*float goertzel_mag(int numSamples,int TARGET_FREQUENCY,int SAMPLING_RATE, float* data)
{
    int     k,i;
    float   floatnumSamples;
    float   omega,sine,cosine,coeff,q0,q1,q2,magnitude,real,imag;

    float   scalingFactor = numSamples / 2.0;

    floatnumSamples = (float) numSamples;
    k = (int) (0.5 + ((floatnumSamples * TARGET_FREQUENCY) / SAMPLING_RATE));
    omega = (2.0 * M_PI * k) / floatnumSamples;
    sine = sin(omega);
    cosine = cos(omega);
    coeff = 2.0 * cosine;
    q0=0;
    q1=0;
    q2=0;

    for(i=0; i<numSamples; i++)
    {
        q0 = coeff * q1 - q2 + data[i];
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


*/
