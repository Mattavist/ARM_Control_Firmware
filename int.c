#include "defs.h"
#include <avr/interrupt.h>

// Timer system interrupt, flips flags
ISR(TIMER0_OVF_vect) {
	twentyFiveMS_Timer--;
	if(!twentyFiveMS_Timer) {  // Fires every 24.576ms
		#ifdef RECEIVER
			if(rcvrFlag)
				rcvrFlag--;
		#endif

		secondTimer--;
		if(!secondTimer) {  // Fires every 1.007616s
			secondTimer = INTS_PER_SECOND;

			if(cal_time)
				cal_time--;

			if(roboteqResponseTime)
				roboteqResponseTime--;

			if (radioAssocTmr)
				radioAssocTmr--;

			if (targetAssocTmr)
				targetAssocTmr--;
		}
		twentyFiveMS_Timer = INTS_PER_25MS;
	}
}

// Wire TTL Rx data interrupt
ISR(USART0_RX_vect) { 
    rxWireFlag = 1;
	wireReceived = UDR0;
	if (wireReceived == RCVR_READY) 
		rcvrFlag = 1;
	else if (wireReceived == START_BYTE)
		startDataFlag = 1;
	else if (wireReceived == CONFIG_CMD)
		configFlag = 1;
	else if (wireReceived == ROBOTEQ_CONFIRM) {
		roboteqFlag = 1;
		targetAssocTmr = TARGET_ASSOC_LIMIT;
	}
}

// Radio TTL Rx data interrupt
ISR(USART1_RX_vect) { 
    rxRadioFlag = 1;
	radioReceived = UDR1;
	if (radioReceived == RCVR_READY) {
		rcvrFlag = 1;
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
		roboteqFlag = 1;
}