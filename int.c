/******************************************************************************
int.c
Written by: Matt Kelly, John Sabino, Jon Wang

Contains timer and USART RX interrupts Specific to ARM Motion Tracker project
******************************************************************************/
#include "defs.h"
#include <avr/interrupt.h>

// Timer system interrupt vector, flips flags
ISR(TIMER0_OVF_vect) {
	twentyMS_Tmr--;
	if(!twentyMS_Tmr) {  // Fires every 20ms
		if(rcvrTmr)
			rcvrTmr--;

		if(roboteqTmr)
			roboteqTmr--;
		
		if (sensorTmr)
			sensorTmr--;

		if(radioTmr)
			radioTmr--;

		if(button1Bouncing)
			button1Bouncing--;

		if(button2Bouncing)
			button2Bouncing--;

		secondTmr--;
		if(!secondTmr) {  // Fires every second
			secondTmr = INTS_PER_SECOND;

			if(calTmr)
				calTmr--;

			if(roboteqResponseTmr)
				roboteqResponseTmr--;

			if (radioAssocTmr)
				radioAssocTmr--;

			if (targetAssocTmr)
				targetAssocTmr--;
		}
		twentyMS_Tmr = INTS_PER_20MS;
	}
}


// Wire TTL Rx data interrupt vector
// Checks for communication flags
// Stores received character in wireReceieved
ISR(USART0_RX_vect) { 
    rxWireFlag = 1;
	wireReceived = UDR0;
	if (wireReceived == RCVR_READY) 
		rcvrReadyFlag = 1;
	else if (wireReceived == START_BYTE)
		startDataFlag = 1;
	else if (wireReceived == CONFIG_CMD)
		configFlag = 1;
	else if (wireReceived == ROBOTEQ_CONFIRM) {
		roboteqTmr = 1;
		targetAssocTmr = TARGET_ASSOC_LIMIT;
	}
}


// Radio TTL Rx data interrupt
// Checks for communication flags
// Stores received character in radioReceieved
ISR(USART1_RX_vect) { 
    rxRadioFlag = 1;
	radioReceived = UDR1;
	if (radioReceived == RCVR_READY) {
		rcvrReadyFlag = 1;
		PORTC |= RADIO_LED;
		radioAssocTmr = RADIO_ASSOC_LIMIT;
	}
	else if (radioReceived == START_BYTE) {
		startDataFlag = 1;
		PORTC |= RADIO_LED;
		radioAssocTmr = RADIO_ASSOC_LIMIT;
	}
	else if (radioReceived == CONFIG_CMD)
		configFlag = 1;
	else if (wireReceived == ROBOTEQ_CONFIRM)
		roboteqTmr = 1;
}