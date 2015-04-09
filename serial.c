/******************************************************************************
serial.c
Written by: Matt Kelly, John Sabino, Jon Wang

Contains code for initialization and serial communication over USART0 and
	USART1 (on ATmega164P). Speeds defined in defs.h.
******************************************************************************/
#include "defs.h"

// WIREINIT FUNCTION
// Initializes USART0 for wired communication
void wireInit() {
	#define BAUD WIRE_SPEED
	#include <util/setbaud.h>
	UBRR0H=UBRRH_VALUE;
	UBRR0L=UBRRL_VALUE;  //set baud rate
	UCSR0B = (1<<TXEN0)|(1<<RXEN0)|(1<<RXCIE0);  //enable receiver and transmitter
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);  // 8bit data format
	#if USE_2X
		UCSR0A |= (1 << U2X0);
	#else
		UCSR0A &= ~(1 << U2X0);
	#endif
	#undef BAUD
}


// WIRESEND FUNCTION
// Sends a character over USART0
void wireSend(char ch) {
		while (!( UCSR0A & (1<<UDRE0)));  // wait while register is free
		UDR0 = ch;
}


// WIRESENDSTRING FUNCTION
// Sends a string over USART0
void wireSendString(char *str) {
	while(*str)
		wireSend(*str++);
}


// RADIOINIT FUNCTION
// Initializes USART1 for radio communication
void radioInit() {
	#define BAUD RADIO_SPEED
	#include <util/setbaud.h>
	UBRR1H=UBRRH_VALUE;
	UBRR1L=UBRRL_VALUE;  //set baud rate
	UCSR1B = (1<<TXEN1)|(1<<RXEN1)|(1<<RXCIE1);  //enable receiver and transmitter
	UCSR1C = (1<<UCSZ10)|(1<<UCSZ11);  // 8bit data format
	#if USE_2X
		UCSR1A |= (1 << U2X1);
	#else
		UCSR1A &= ~(1 << U2X1);
	#endif
	#undef BAUD
}


// RADIOSEND FUNCTION
// Sends a character over USART0
void radioSend(char ch) {
		while (!( UCSR1A & (1<<UDRE1)));  // wait while register is free
		UDR1 = ch;
}


// RADIOSENDSTRINGFUNCTION
// Sends a string over USART1
void radioSendString(char *str) {
	while(*str)
		radioSend(*str++);
}




