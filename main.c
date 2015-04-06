#define MAIN
#include "defs.h"
#include "adc.h"
#include "roboteq.h"
#include "serial.h"
#include "timer.h"
#include <string.h>
#include <avr/interrupt.h>
#include <stdio.h>

// Function Prototypes
void sampleSensorData();
void transmitSensorData(char *, int);
int receiveSensorData(char *, int);
void dataToTerminal();
int basicRoboControl();
int roboControl();
void processButtons();

// SYSINIT FUNCTION
// Sets port data directions and initializes systems
void sysInit() {
	DDRA = 0x15;		// A0,2,4 as output
	PORTA = 0xC0;		// enable A6-7's pull-up resistors
	DDRB = 0x00;		// B0-7 as input
	PORTB = 0xFF;		// Enable B's pull-up resistors
	DDRC = 0x0F;		// Set PortC 0-3 as output
	PORTC = 0xF0;		// Enable PortC's pull-up resistors
	DDRD  = 0x80;   	// D3 as output
	PORTD |= 0x80;  	// Turn on the power LED

	rcvrTmr = 0;
	configFlag = 0;
	rxRadioFlag = 0;
	rxWireFlag = 0;
	startDataFlag = 0;
	targetIsRoboteQ = TARGET;
	twentyMS_Tmr = INTS_PER_20MS;
	secondTmr = INTS_PER_SECOND;
	roboteqResponseTmr = 0;
	roboteqStatus = 0;
	roboteqErrCnt = 0;
	button1Bouncing = 0;
	button2Bouncing = 0;

	adcInit();
	wireInit();
	radioInit();
	timerInit();
	wireSendString("Starting\r\n");
	sei();
	PORTC |= POWER_LED;
}


// MAIN FUNCTION
// Initializes hardware
// Runs main program loop for Transmitter or Receiver, chosen at compile
int main() {
	sysInit();

	// TRANSMITTER MAIN PROGRAM LOOP
	#ifdef TRANSMITTER 
	while(1) {
		// TODO: This happens every time through the loop, too much!
		// Replace with timer trigger
		sampleSensorData(); // get 6 adc values and stick them in the buffer

		// Ready command from receiver?
		if (rcvrTmr == 1) { 
			transmitSensorData(data, NUM_ADC_CHANS + NUM_DIGITAL_CHANS);  // transmit the data buffer
			rcvrTmr = 0;  // set the ready flag low
		}
		
		// received config command from terminal? Does nothing currently
		if (configFlag == 1) {
			configFlag = 0;  // set the config flag low
		}

		processButtons();
	}
	#endif

	// RECEIVER MAIN PROGRAM LOOP
	#ifdef RECEIVER
	
	while(1) {
		// Make sure RoboteQ is initialized and connected
		if (roboteqErrCnt > ROBOTEQ_ERROR_LIMIT) {
			PORTC &= ~TARGET_LED;
			roboteqStatus = 0;
		}

		// Turn off the associate LED if timer expires
		if (!radioAssocTmr)
			PORTC &= ~RADIO_LED;

		// Connect to and initialize the RoboteQ
		while(roboteqStatus == 0 && targetIsRoboteQ)
			roboteqInit();


		if(!roboteqTmr && !targetIsRoboteQ) {
			if(dataValid) {
				dataToTerminal();
				dataValid = 0;
			}
			roboteqTmr = ROBOTEQ_DELAY;
		}

		// Communicate with RoboteQ regularly
		if(!roboteqTmr && targetIsRoboteQ) {
			if (dataValid) {
				//if(!roboControl())
				if(!basicRoboControl())
					roboteqErrCnt++;
				dataValid = 0;
			}
			else  // Feed the watchdog
				dataToRoboteq(PING);
			roboteqTmr = ROBOTEQ_DELAY;
		}

		// Ready to receive data?
		if (!rcvrTmr) {
			if (!targetIsRoboteQ) {
				wireSendString("Requesting data...\r\n");
			}
			radioSend(RCVR_READY);
			rcvrTmr = RCVR_DELAY;
		}

		// Start byte from transmitter?
		if (startDataFlag == 1) {
			if (receiveSensorData(data, NUM_ADC_CHANS + NUM_DIGITAL_CHANS))
				dataValid = 1;
			else
				dataValid = 0;
			startDataFlag = 0;
		}

		// command byte from transmitter?
		if (configFlag == 1) {
			// command roboteq to stop
			// process the command
			// allow roboteq to resume
		}
	}
	#endif

	return 0;
}


// sampleSensorData FUNCTION
// Collects 3 ADC readings, 2 digital readings, and populates the buffer
// Returns nothing
void sampleSensorData() {
	PORTA = 0xC1;  // Power to ADC channel 1
	data[0] = getADC(1)/4;

	PORTA = 0xC4;  // Power to ADC channel 3
	data[1] = getADC(3)/4;

	PORTA = 0xD0;  // Power to ADC channel 5
	data[2] = getADC(5)/4;
	PORTA = 0xC0;

	data[3] = (PINA & 0x40)?1:0;
	data[4] = (PINA & 0x80)?1:0;
}


// transmitSensorData FUNCTION
// Sends start byte, size unsigned chars, and a checksum of the added chars
// Returns nothing
// TODO: Switch to choose start byte
void transmitSensorData(char *buffer, int size) {
	unsigned char checksum = 0;

	radioSend(START_BYTE);
	for (int i = 0; i < size; i++) {
		radioSend(buffer[i]);
		wireSend(buffer[i]);
		checksum += buffer[i]; 
	}
	radioSend(checksum);
}


// receiveSensorData FUNCTION
// Receives size chars and places them into a buffer
// Compares the sum of the chars against the checksum
// Returns 1 if checksum match, 0 otherwise
// Returns 0 if times out
int receiveSensorData(char *buffer, int size) {
	unsigned char checksum = 0;
	rxRadioFlag = 0;
	radioTmr = RADIO_TIMEOUT;

	// Populate the buffer
	for (int i = 0; i < size; i++) {
		while(!rxRadioFlag)
			if(!radioTmr) return 0;
		buffer[i] = radioReceived;
		checksum += radioReceived;
		rxRadioFlag = 0;
	}

	// Validate against checksum
	while(!rxRadioFlag)
			if(!radioTmr) return 0;
	rxRadioFlag = 0;
	
	if (checksum != radioReceived)
		return 0;
	else
		return 1;
}


// DATATOTERMINAL FUNCTION
// Prints the sensor data to terminal in a nice format
// Returns nothing
void dataToTerminal() {
	char buf[80];

	wireSendString("Got good data!\r\n");
	data[0] -= 60;
	data[0] = data[0]*100/91;			
	sprintf(buf, "ADC0    = %d%c\r\n", data[0], '%');
	wireSendString(buf);
	data[1] -= 60;
	data[1] = data[1]*100/91;
	sprintf(buf, "ADC1    = %d%c\r\n", data[1], '%');
	wireSendString(buf);
	data[2] = data[2]*100/255;
	sprintf(buf, "ADC2    = %d%c\r\n", data[2], '%');
	wireSendString(buf);
	sprintf(buf, "Switch0 = %s\r\n", (data[3])?"Open":"Closed");
	wireSendString(buf);
	sprintf(buf, "Switch1 = %s\r\n\n", (data[4])?"Open":"Closed");
	wireSendString(buf);
}


int basicRoboControl() {
	setRoboPower(1, 20);
	setRoboPosition(1, 50);
	//setRoboPower(2, 20);
	//setRoboPosition(2, 75);
	setRoboPower(1, 20);
	setRoboPosition(1, 25);
	return 1;
}

int roboControl() {
	if(!setRoboPower(1, 20))
		return 0;
	if(!setRoboPosition(1, data[0]))
		return 0;
	if(!setRoboPower(2, 20))
		return 0;
	if(!setRoboPosition(2, data[1]))
		return 0;
	if(!setRoboPower(3, 20))
		return 0;
	if(!setRoboPosition(3, data[2]))
		return 0;

	return 1;
}

void processButtons() {
	if ((BUTTON1_PRESSED) && (button1State == 0)) {// && !button1Bouncing) {  // Handle first button down
		if(!button1Bouncing) {
			button1Bouncing = BOUNCE_TIME;
			button1State = 1;
		}
		// set this button's flag
	}
	else if (!BUTTON1_PRESSED && (button1State == 2)) {
		if(!button1Bouncing) {
			button1Bouncing = BOUNCE_TIME;
			button1State = 0;
		}
	}

	if (button1State == 1) {
		//do calibration
		button1State = 2;
	}


	if ((BUTTON2_PRESSED) && (button2State == 0)) {// && !button1Bouncing) {  // Handle first button down
		if(!button2Bouncing) {
			button2Bouncing = BOUNCE_TIME;
			button2State = 1;
		}
		// set this button's flag
	}
	else if (!BUTTON2_PRESSED && (button2State == 2)) {
		if(!button2Bouncing) {
			button2Bouncing = BOUNCE_TIME;
			button2State = 0;
		}
	}

	if (button2State == 1) {
		//reset?
		button2State = 2;
	}
}


