/******************************************************************************
main.c
Written by: Matt Kelly, John Sabino, Jon Wang

Contains main program loops for ARM Motion Tracker Project
Code for both receiver and transmitter modules. Switch between the two
	in defs.h
******************************************************************************/
#define MAIN
#include "defs.h"
#include "adc.h"
#include "roboteq.h"
#include "serial.h"
#include "timer.h"
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


// Function Prototypes
void sampleSensorData();
void transmitSensorData(char *, int);
int  receiveSensorData(char *, int);
void dataToTerminal();
int  basicRoboControl();
int  roboControl();
void processButtons();
void initSensors();
void calibration();

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
	sensorTmr = 0;
	configFlag = 0;
	rxRadioFlag = 0;
	rxWireFlag = 0;
	calFlag = 0;
	resetFlag = 0;
	startDataFlag = 0;
	rcvrReadyFlag = 0;
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
	initSensors();
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
		wireSendString("Hello\r\n");
		wireSendString("Hello\r\n");

		while(1) {
			// Turn off the associate LED if timer expires
			if (!radioAssocTmr)
				PORTC &= ~RADIO_LED;

			// Sample sensor data
			if (!sensorTmr) {
				//wireSendString("Sampling...\r\n");
				sampleSensorData(); // get 6 adc values and stick them in the buffer
				sensorTmr = SENSOR_DELAY;
			}

			// Ready command from receiver?
			if (rcvrReadyFlag == 1) { 
				transmitSensorData(data, NUM_ADC_CHANS + NUM_DIGITAL_CHANS);  // transmit the data buffer
				rcvrReadyFlag = 0;  // set the ready flag low
			}

			processButtons();
			if(calFlag)
				calibration();

			if(resetFlag) {
				eeprom_update_byte((uint8_t*)0x01, 0x00);
				sysInit();
			}
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
					//dataToTerminal();
					basicRoboControl();
					dataValid = 0;
				}
				roboteqTmr = ROBOTEQ_DELAY;
			}

			// Communicate with RoboteQ regularly
			if(!roboteqTmr && targetIsRoboteQ) {
				if (dataValid) {
					//TODO CHANGE THIS TO ROBOCONTROL
					//if(!roboControl())
					if(!basicRoboControl())
						roboteqErrCnt++;
					dataValid = 0;
				}
				else  // Feed the watchdog
					if (DEBUG) radioSendString("Pinging RoboteQ\r\n");
					dataToRoboteq(PING);
				roboteqTmr = ROBOTEQ_DELAY;
			}

			// Ready to receive data?
			if (!rcvrTmr) {
				if (!targetIsRoboteQ) {
					//wireSendString("Requesting data...\r\n");
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
		}
	#endif

	return 0;
}


// SAMPLESENSORDATA FUNCTION
// Collects 3 ADC readings, 2 digital readings, and populates the buffer
// Returns nothing
void sampleSensorData() {
	unsigned char temp;
	char buf[100];

	PORTA = 0xC1;  // Power to ADC channel 1
	temp = (getADC(1)/4)*100/(dataMax[0] - dataMin[0]);
	temp = ((avgADC(1)/4)-dataMin[0])*100/(dataMax[0] - dataMin[0]);
	if (temp > data[0] + 1 || temp < data[0] - 1) {
		data[0] = temp;
	}

	PORTA = 0xC4;  // Power to ADC channel 3
	temp = ((avgADC(3)/4)-dataMin[1])*100/(dataMax[1] - dataMin[1]);
	if (temp > data[1] + 1 || temp < data[1] - 1) {
		data[1] = temp;
	}

	PORTA = 0xD0;  // Power to ADC channel 5
	temp = ((avgADC(5)/4)-dataMin[2])*100/(dataMax[2] - dataMin[2]);
	if (temp > data[2] + 1 || temp < data[2] - 1) {
		data[2] = temp;
	}
	PORTA = 0xC0;

	data[3] = (PINA & 0x40)?1:0;
	data[4] = (PINA & 0x80)?1:0;

	if (DEBUG) dataToTerminal();
}


// TRANSMITSENSORDATA FUNCTION
// Sends start byte, size unsigned chars, and a checksum of the added chars
// Returns nothing
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


// RECEIVESENSORDATA FUNCTION
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
void dataToTerminal() {
	char buf[80];

	sprintf(buf, "Min0: %d, Max0: %d\r\n", dataMin[0], dataMax[0]);
	wireSendString(buf);
	sprintf(buf, "Min1: %d, Max1: %d\r\n", dataMin[1], dataMax[1]);
	wireSendString(buf);
	sprintf(buf, "Min2: %d, Max2: %d\r\n\r\n", dataMin[2], dataMax[2]);
	wireSendString(buf);

	/*wireSendString("Got good data!\r\n");
	sprintf(buf, "ADC0    = %d%c\r\n", data[0], '%');
	wireSendString(buf);
	sprintf(buf, "ADC1    = %d%c\r\n", data[1], '%');
	wireSendString(buf);
	sprintf(buf, "ADC2    = %d%c\r\n", data[2], '%');
	wireSendString(buf);
	sprintf(buf, "Switch0 = %s\r\n", (data[3])?"Open":"Closed");
	wireSendString(buf);
	sprintf(buf, "Switch1 = %s\r\n\n", (data[4])?"Open":"Closed");
	wireSendString(buf);*/
}


// BASICROBOCONTROL FUNCTION
// Generates terminal strings for sensor data
int basicRoboControl() {
	char buf[100];

	// Funky math here is required for proper calculation
	sprintf(buf, "!G 2 %d\r\n", (data[2]*20)-1000);
	wireSendString(buf);
	return 1;
}


// ROBOCONTROL FUNCTION
// Generates and sends commands to RoboteQ based on sensor data
int roboControl() {
	if(!setRoboPower(1, 20))
		return 0;
	if(!setRoboPosition(1, (data[0]*20)-1000))
		return 0;
	if(!setRoboPower(2, 20))
		return 0;
	if(!setRoboPosition(2, (data[1]*20)-1000))
		return 0;
	if(!setRoboPower(3, 20))
		return 0;
	if(!setRoboPosition(3, (data[2]*20)-1000))
		return 0;

	return 1;
}


// PROCESSBUTTONS FUNCTION
// State machine to process and debounce button presses
void processButtons() {
	if ((BUTTON1_PRESSED) && (button1State == 0)) {
		if(!button1Bouncing) {
			button1Bouncing = BOUNCE_TIME;
			calFlag = !calFlag;
			button1State = 1;
		}
	}
	else if (!BUTTON1_PRESSED && (button1State == 2)) {
		if(!button1Bouncing) {
			button1Bouncing = BOUNCE_TIME;
			button1State = 0;
		}
	}

	if (button1State == 1) {
		if (DEBUG) wireSendString("Calibration Button pressed\r\n");
		button1State = 2;
	}


	if ((BUTTON2_PRESSED) && (button2State == 0)) {
		if(!button2Bouncing) {
			button2Bouncing = BOUNCE_TIME;
			resetFlag = 1;
			button2State = 1;
		}
	}
	else if (!BUTTON2_PRESSED && (button2State == 2)) {
		if(!button2Bouncing) {
			button2Bouncing = BOUNCE_TIME;
			button2State = 0;
		}
	}

	if (button2State == 1) {
		if (DEBUG) wireSendString("Reset Button pressed\r\n");
		button2State = 2;
	}
}


// CALIBRATION FUNCTION
// Runs for CAL_TIME seconds constantly grabbing
// maximum and minimum values on all ADC sensor channels
void calibration() {
	unsigned char temp[3];// Initial values and variables
	PORTC |= MISC_LED;	// Turn on LED, Start calibration
	PORTA = 0xD5;  // Turn on power to all ADC channels

	dataMin[0] = 255;
	dataMax[0] = 0;
	dataMin[1] = 255;
	dataMax[1] = 0;
	dataMin[2] = 255;
	dataMax[2] = 0;

	while (calFlag) {
		for(int i = 0; i < NUM_ADC_CHANS; i++) {
			temp[i] = avgADC(i*2 + 1)/4;
			if (temp[i] > dataMax[i])
				dataMax[i] = temp[i];
			else if (temp[i] < dataMin[i])
				dataMin[i] = temp[i];
		}
		processButtons();
	}

	for(int i = 0; i < NUM_ADC_CHANS; i++) {
			eeprom_update_byte((uint8_t*)(i+2), dataMin[i]);
			eeprom_update_byte((uint8_t*)(i+10), dataMax[i]);
		}
	eeprom_update_byte((uint8_t*)0x01, 0x01);

	PORTA = 0xC0;
	PORTC &= ~MISC_LED;	// Turn off LED, End calibration
}


// INITSENSORS FUNCTION
// Currently only hard coded sensor limits
// TODO: Change to reading from EEPROM if it has valid data
void initSensors() {
	if (eeprom_read_byte((uint8_t*)1)) {
		for(int i = 0; i < NUM_ADC_CHANS; i++) {
			dataMin[i] = eeprom_read_byte((uint8_t*)(i+2));
			dataMax[i] = eeprom_read_byte((uint8_t*)(i+10));
		}
	}
	else {
		dataMin[0] = 60;
		dataMax[0] = 151;
		dataMin[1] = 60;
		dataMax[1] = 151;
		dataMin[2] = 0;
		dataMax[2] = 255;
	}
}