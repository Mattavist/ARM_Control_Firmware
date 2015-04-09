/******************************************************************************
roboteq.c
Written by: Matt Kelly, John Sabino, Jon Wang

Contains code for initialization of timer interrupts.
******************************************************************************/
#include "defs.h"

// TIMERINIT FUNCTION
// Initializes 8-bit Timer0
void timerInit() {
	TCCR0B=(1<<CS01)|(1<<CS00);  // Generate an interrupt every 20ms
	TIMSK0 |=(1<<TOIE0);  // Turn on Timer0's interrupt
}