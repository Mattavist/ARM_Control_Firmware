#include "defs.h"
#include "serial.h"
#include <stdio.h>


// GETROBOTEQCONFIRM FUNCTION
int getRoboteqConfirm() {	
	roboteqResponseTime = TARGET_ASSOC_LIMIT;
	rxWireFlag = 0;
	while(!rxWireFlag) {
		if (!roboteqResponseTime) {
			radioSendString("Command not acknowledged!\r\n");
			roboteqErrCnt++;
			return 0;
		}
	}

	rxWireFlag = 0;
	radioSend(wireReceived);
	if (wireReceived == ROBOTEQ_CONFIRM) {
		radioSendString("RoboteQ confirmed!\r\n");
		roboteqErrCnt = 0;
		//PORTC |= TARGET_LED;
		return 1;
	}
	else {
		radioSendString("Bad response!\r\n");
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

int setRoboPosition(int chan, int pos) {
	char buf[100];
	sprintf(buf, "!G %d %d_", chan, pos*10);
	wireSendString(buf);
	return getRoboteqConfirm();
}

int setRoboPower(int chan, int pow) {
	char buf[100];
	sprintf(buf, "!P %d %d_", chan, pow*10);
	wireSendString(buf);
	return getRoboteqConfirm();
}

// ROBOTEQINIT FUNCTION
// Queries Roboteq for model number and initializes hardware
// Returns 1 if successful, 0 otherwise
int roboteqInit() {
	int response;

	PORTC &= ~TARGET_LED;
	roboteqFlag = 0;

	/*
	radioSendString("?fid_");

	rxWireFlag = 0;
	wireSendString("?fid_");
	roboteqResponseTime = TARGET_ASSOC_LIMIT;
	response = wireGetCmpString(&roboteqResponseTime, ROBOTEQ_MODEL);
	//response = 1;
	
	//PORTC |= TARGET_LED;
	//while(1);
	if (response == 1) {
		radioSendString("Success!\r\n");
	}
	else if (response == -1) {
		radioSendString("Bad string response\r\n");
		return 0;
	}
	else {
		radioSendString("Time out!\r\n");
		return 0;
	}

	while(1);*/

	if (!dataToRoboteq("^ECHOF 1_")) return 0;
	if (!setRoboPower(1, 50)) return 0;
	if (!setRoboPosition(1, 50)) return 0;
	roboteqStatus = 1;
	roboteqErrCnt = 0;
	PORTC |= TARGET_LED;

	return 1;
}


