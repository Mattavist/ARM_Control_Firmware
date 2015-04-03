// uC definitions
#define F_CPU 7372800UL	 // set the CPU clock
#include <avr/io.h>

// Module Definition
//#define TRANSMITTER
#define RECEIVER

// Flags and comands
#define DELAY 10
#define BUTTON_PRESSED !(PINC & 0x10)  // Change to specific button
#define POWER_LED 0x01
#define RADIO_LED 0x02
#define TARGET_LED 0x04
#define	START_BYTE 0xDB  // Sensor Data start command
#define RCVR_READY 0x4D  // Receiver ready command
#define CONFIG_CMD 0xB3  // Command for receiver to enter config mode
#define NUM_READINGS 3  // Number of ADC readings to be averaged
#define INTS_PER_25MS 3  // Interrupts per 10ms
#define INTS_PER_SECOND 41  // MS counts per second
#define NUM_ADC_CHANS 3  // Number of attached ADC sensors
#define NUM_DIGITAL_CHANS 2  // Number of attached digital sensors
#define RADIO_ASSOC_LIMIT 2
#define TARGET_ASSOC_LIMIT 5
#define DEBOUNCE_COUNT 100  // Counter for switch debouncing
#define ROBOTEQ_CONFIRM '+'
#define ROBOTEQ 1
#define TERMINAL 2
#define ROBOTEQ_MODEL "FID=Roboteq v1.3 RCB290 10/10/2013"
#define ROBOTEQ_ERROR_LIMIT 10
#define RCVR_DELAY 3
#define ROBOTEQ_DELAY 4

// Shared Variables
#ifdef MAIN
	volatile unsigned char rxWireFlag;  // Data ready in wired USART RX buffer
	volatile unsigned char rxRadioFlag;  // Data ready in wireless USART RX buffer
	volatile unsigned char radioReceived;  // Data from USART RX Radio
	volatile unsigned char wireReceived;  // Data from USART RX Wire
	volatile unsigned char rcvrFlag;  // Receiver is ready for sensor data
	volatile unsigned char configFlag;  // Received command to enter config mode
	volatile unsigned char startDataFlag;  // About to receive sensor data
	volatile unsigned char roboteqFlag;
	volatile unsigned int twentyFiveMS_Timer;
	volatile unsigned int secondTimer;
	volatile unsigned int cal_time;
	volatile unsigned int roboteqResponseTime;
	volatile unsigned int radioAssocTmr;
	volatile unsigned int targetAssocTmr;
	volatile unsigned int roboteqStatus;
	volatile unsigned int roboteqErrCnt;
	unsigned char data[NUM_ADC_CHANS + NUM_DIGITAL_CHANS];  // Buffer with sensor data
	unsigned char dataMin[NUM_ADC_CHANS];
	unsigned char dataMax[NUM_ADC_CHANS];
	unsigned char targetDevice;
#else
	extern volatile unsigned char rxWireFlag;  // Data ready in wired USART RX buffer
	extern volatile unsigned char rxRadioFlag;  // Data ready in wireless USART RX buffer
	extern volatile unsigned char radioReceived;  // Data from USART RX Radio
	extern volatile unsigned char wireReceived;  // Data from USART RX Wire
	extern volatile unsigned char rcvrFlag;  // Receiver is ready for sensor data
	extern volatile unsigned char configFlag;  // Received command to enter config mode
	extern volatile unsigned char startDataFlag;  // About to receive sensor data
	extern volatile unsigned char roboteqFlag;
	extern volatile unsigned int twentyFiveMS_Timer;
	extern volatile unsigned int secondTimer;
	extern volatile unsigned int cal_time;
	extern volatile unsigned int roboteqResponseTime;
	extern volatile unsigned int radioAssocTmr;
	extern volatile unsigned int targetAssocTmr;
	extern volatile unsigned int roboteqStatus;
	extern volatile unsigned int roboteqErrCnt;
	extern unsigned char data[NUM_ADC_CHANS + NUM_DIGITAL_CHANS];  // Buffer with sensor data
	extern unsigned char dataMin[NUM_ADC_CHANS];
	extern unsigned char dataMax[NUM_ADC_CHANS];
	extern unsigned char targetDevice;
#endif