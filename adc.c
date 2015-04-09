/******************************************************************************
adc.c
Written by: Matt Kelly, John Sabino, Jon Wang

Contains functions to initialize and collect ADC readings
******************************************************************************/
#include "defs.h"

// ADCINIT FUNCTION
// Enable ADC and prescale clock to 125kHz for 8mHz clock
void adcInit() {
	ADCSRA  = (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);
	ADMUX = (1 << REFS0); // Disconnect AREF, use AVCC as reference
}


// GETADC FUNCTION
// Returns one ADC reading from desired channel
int getADC(int chan) {
	ADMUX = (1 << REFS0) | chan;	// Select channel for ADC reading
	ADCSRA  |= (1<<ADSC);  // Start conversion
	while (ADCSRA & (1<<ADSC));  // Wait for completion

	return ADCW;	// Store ADC result
}


// AVGADC FUNCTION
// Samples chan for NUM_READINGS ADC values and averages them
int avgADC(int chan) {
	int i, value = 0;

	for(i = 0; i<NUM_READINGS; i++) {
		value += getADC(chan);
	}

	return value/NUM_READINGS;
}