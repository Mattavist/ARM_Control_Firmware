#include "defs.h"

// Generates timer interrupt 122 times per second
void timerInit() {
	TCCR0B=(1<<CS01)|(1<<CS00);
	TIMSK0 |=(1<<TOIE0);
}