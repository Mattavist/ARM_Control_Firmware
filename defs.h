// uC definitions
#define F_CPU 7372800UL	 // set the CPU clock
#include <avr/io.h>

// Module Definition
#define TRANSMITTER
//#define RECEIVER
#define TARGET 0

// Flags and comands
#define DELAY 10
#define BUTTON1_PRESSED !(PINC & 0x10)  // Change to specific button
#define BUTTON2_PRESSED !(PINC & 0x20)  // Change to specific button
#define POWER_LED 0x01  // Power LED at C0
#define RADIO_LED 0x02  // Radio LED at C1
#define TARGET_LED 0x04  // Target device LED at C2
#define	START_BYTE 0xDB  // Sensor Data start command
#define RCVR_READY 0x4D  // Receiver ready command
#define CONFIG_CMD 0xB3  // Command for receiver to enter config mode
#define NUM_READINGS 3  // Number of ADC readings to be averaged
#define INTS_PER_20MS 9  // Interrupts per 10ms
#define INTS_PER_SECOND 50  // MS counts per second
#define NUM_ADC_CHANS 3  // Number of attached ADC sensors
#define NUM_DIGITAL_CHANS 2  // Number of attached digital sensors
#define RADIO_ASSOC_LIMIT 2  // Timer for Radio LED
#define TARGET_ASSOC_LIMIT 5  // Timer for Target LED
#define DEBOUNCE_COUNT 100  // Counter for switch debouncing
#define ROBOTEQ_CONFIRM '+'  // Confirm character from RoboteQ
#define ROBOTEQ 1  // RoboteQ identifier
#define TERMINAL 2  // Terminal identifier
#define ROBOTEQ_MODEL "FID=Roboteq v1.3 RCB290 10/10/2013"  // Motor controller firmware version
#define ROBOTEQ_ERROR_LIMIT 10  // Number of errors before reinitialization
#define RCVR_DELAY 4  // t*20ms delay for sensor data requests
#define RADIO_TIMEOUT 3  // t*20ms communication timer
#define ROBOTEQ_DELAY 4  // t*20ms delay for roboteq communication
#define PING "^ECHOF 1_"  // Empty command for RoboteQ
#define WIRE_SPEED 115200
#define RADIO_SPEED 57600
#define BOUNCE_TIME 1

// Shared Variables
#ifdef MAIN
	volatile unsigned char rxWireFlag;  // Data ready in wired USART RX buffer
	volatile unsigned char rxRadioFlag;  // Data ready in wireless USART RX buffer
	volatile unsigned char radioReceived;  // Data from USART RX Radio
	volatile unsigned char wireReceived;  // Data from USART RX Wire
	volatile unsigned char rcvrTmr;  // Receiver is ready for sensor data
	volatile unsigned char configFlag;  // Received command to enter config mode
	volatile unsigned char startDataFlag;  // About to receive sensor data
	volatile unsigned char roboteqTmr;
	volatile unsigned int twentyMS_Tmr;
	volatile unsigned int secondTmr;
	volatile unsigned int calTmr;
	volatile unsigned int roboteqResponseTmr;
	volatile unsigned int radioAssocTmr;
	volatile unsigned int targetAssocTmr;
	volatile unsigned int roboteqStatus;
	volatile unsigned int roboteqErrCnt;
	volatile unsigned int radioTmr;
	volatile unsigned int button1Bouncing;
	volatile unsigned int button2Bouncing;
	volatile unsigned int button1State;
	volatile unsigned int button2State;
	unsigned char data[NUM_ADC_CHANS + NUM_DIGITAL_CHANS];  // Buffer with sensor data
	unsigned char dataMin[NUM_ADC_CHANS];
	unsigned char dataMax[NUM_ADC_CHANS];
	unsigned char targetIsRoboteQ;
	unsigned char dataValid;
#else
	extern volatile unsigned char rxWireFlag;  // Data ready in wired USART RX buffer
	extern volatile unsigned char rxRadioFlag;  // Data ready in wireless USART RX buffer
	extern volatile unsigned char radioReceived;  // Data from USART RX Radio
	extern volatile unsigned char wireReceived;  // Data from USART RX Wire
	extern volatile unsigned char rcvrTmr;  // Receiver is ready for sensor data
	extern volatile unsigned char configFlag;  // Received command to enter config mode
	extern volatile unsigned char startDataFlag;  // About to receive sensor data
	extern volatile unsigned char roboteqTmr;
	extern volatile unsigned int twentyMS_Tmr;
	extern volatile unsigned int secondTmr;
	extern volatile unsigned int calTmr;
	extern volatile unsigned int roboteqResponseTmr;
	extern volatile unsigned int radioAssocTmr;
	extern volatile unsigned int targetAssocTmr;
	extern volatile unsigned int roboteqStatus;
	extern volatile unsigned int roboteqErrCnt;
	extern volatile unsigned int radioTmr;
	extern volatile unsigned int button1Bouncing;
	extern volatile unsigned int button2Bouncing;
	extern volatile unsigned int button1State;
	extern volatile unsigned int button2State;
	extern unsigned char data[NUM_ADC_CHANS + NUM_DIGITAL_CHANS];  // Buffer with sensor data
	extern unsigned char dataMin[NUM_ADC_CHANS];
	extern unsigned char dataMax[NUM_ADC_CHANS];
	extern unsigned char targetIsRoboteQ;
	extern unsigned char dataValid;
#endif