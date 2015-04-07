#include "defs.h"
#include "serial.h"
#include <stdio.h>


// GETROBOTEQCONFIRM FUNCTION
int getRoboteqConfirm() {	
	roboteqResponseTmr = TARGET_ASSOC_LIMIT;
	rxWireFlag = 0;
	while(!rxWireFlag) {
		if (!roboteqResponseTmr) {
			//radioSendString("Command not acknowledged!\r\n");
			return 0;
		}
	}

	rxWireFlag = 0;
	radioSend(wireReceived);
	if (wireReceived == ROBOTEQ_CONFIRM) {
		//radioSendString("RoboteQ confirmed!\r\n");
		roboteqErrCnt = 0;
		//PORTC |= TARGET_LED;
		return 1;
	}
	else {
		//radioSendString("Bad response!\r\n");
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
	roboteqTmr = 0;

	
	//radioSendString("Querying RoboteQ Firmware... ");

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


