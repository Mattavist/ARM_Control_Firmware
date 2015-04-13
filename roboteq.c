/******************************************************************************
roboteq.c
Written by: Matt Kelly, John Sabino, Jon Wang

Contains code for initialization and serial communication with RoboteQ motor 
controllers. Set the desired controller's ID in defs.h.
******************************************************************************/
#include "defs.h"
#include "serial.h"
#include "roboteq.h"
#include <string.h>
#include <stdio.h>

// GETROBOTEQCONFIRM FUNCTION
// Waits for a response from the RoboteQ
// Returns 1 if command was acknowledged
// Returns 0 if time expired or bad response recieved
int getRoboteqConfirm() {	
	roboteqResponseTmr = TARGET_ASSOC_LIMIT;
	rxWireFlag = 0;
	while(!rxWireFlag) {
		if (!roboteqResponseTmr) {
			if (DEBUG) radioSendString("Command not acknowledged!\r\n");
			roboteqErrCnt++;
			return 0;
		}
	}

	rxWireFlag = 0;
	if (DEBUG) radioSend(wireReceived);
	if (wireReceived == ROBOTEQ_CONFIRM) {
		if (DEBUG) radioSendString("RoboteQ confirmed!\r\n");
		roboteqErrCnt = 0;
		//PORTC |= TARGET_LED;
		return 1;
	}
	else {
		if (DEBUG) radioSendString("Bad response!\r\n");
		roboteqErrCnt++;
		return 0;
	}

}


// DATATOROBOTEQ FUNCTION
// Forms strings from sensor data and transmits to Roboteq
// Returns 1 if data acknowledged, 0 otherwise
int dataToRoboteq(char *str) {
	wireSendString(str);
	return getRoboteqConfirm();
}


// SETROBOTPOSITION FUNCTION
// Sends command to the RoboteQ to move the desired motor
// Returns 1 if successful, 0 otherwise
int setRoboPosition(int chan, int pos) {
	char buf[100];
	sprintf(buf, "!G %d %d_", chan, pos);
	wireSendString(buf);
	return getRoboteqConfirm();
}


// SETROBOTPOWER FUNCTION
// Sends command to the RoboteQ to set the desired motor's speed
// Returns 1 if successful, 0 otherwise
int setRoboPower(int chan, int pow) {
	char buf[100];
	sprintf(buf, "!P %d %d_", chan, pow);
	wireSendString(buf);
	return getRoboteqConfirm();
}


// ROBOTEQINIT FUNCTION
// Queries Roboteq for model number and initializes hardware
// Returns 1 if successful, 0 otherwise
int roboteqInit() {
	int response;

	PORTC &= ~TARGET_LED;
	roboteqTmr = 0;

	
	if (!dataToRoboteq("^ECHOF 1_")) return 0;
	radioSendString("Querying RoboteQ Firmware...\r\n ");

	rxWireFlag = 0;
	wireSendString("?fid_");
	roboteqResponseTmr = TARGET_ASSOC_LIMIT;
	if(wireGetCmpString(&roboteqResponseTmr, ROBOTEQ_MODEL) != 1)
		return 0;

	if (!dataToRoboteq("^ECHOF 1_")) return 0;
	//if (!setRoboPower(1, 50)) return 0;
	//if (!setRoboPosition(1, 50)) return 0;
	roboteqStatus = 1;
	roboteqErrCnt = 0;
	roboteqTmr = ROBOTEQ_DELAY;
	PORTC |= TARGET_LED;

	return 1;
}


// WIREGETCMPSTRING FUNCTION
// Receives a string of characters over USART0 and checks against str
// Returns 1 if strings match, -1 if they don't, 0 if time out
int wireGetCmpString(volatile unsigned int *timer, char str[]) {
	char robo[100] = "";
	int counter = 0;

	while (*timer) {
		if (rxWireFlag) {
			if (wireReceived == '\r') {
				robo[counter] = 0x00;
				rxWireFlag = 0;
				if(!strcmp(robo, str)) {
					rxWireFlag = 0;
					radioSendString(robo);
					radioSendString("\r\n");
					return 1;
				}
				else {
					rxWireFlag = 0;
					radioSendString(robo);
					while(1);
					return -1;
				}
			}
			else {
				rxWireFlag = 0;
				robo[counter++] = wireReceived;
			}
		}
	}
	radioSendString("Timeout\r\n");
	rxWireFlag = 0;
	return 0;
}