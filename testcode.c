	// Contains code for testing
	// Don't try to compile this won't work!

	// Terminal Testing
	#ifdef SERIAL
	while(1) {
		while(!BUTTON_PRESSED) {
			if (rxWireFlag) {
				if (received == 0x08) {
					if (rxIndex > 0) {
						//radioSend(0x08);
						//radioSend(' ');
						//radioSend(0x08);
						rxIndex--;
					}
				}
				else if (received == 0x0D || received == '\n') {
					rxBuf[rxIndex] = 0;
					//radioSend(0x0D);
					//radioSend(0x0A);
					//radioSendString(rxBuf);
					//radioSend(0x0D);
					//radioSend(0x0A);
					wireSend(0x0D);
					wireSend(0x0A);
					wireSendString(rxBuf);
					wireSend(0x0D);
					wireSend(0x0A);
					if (rxIndex == 2 && rxBuf[0] == 't')
						if(rxBuf[1] == 'u')
							PORTC ^= 0x02; 


					rxIndex = 0;
					rxBuf[rxIndex] = 0;
				}
				else {
					rxBuf[rxIndex] = received;
					//radioSend(rxBuf[rxIndex]);
					wireSend(rxBuf[rxIndex]);
					rxIndex++;
				}

				received = 0x00;
				rxWireFlag = 0;
			}
		}
		processButton();
		wireSendString("Hello my name is Matt\r\n");
		wireSendString("...and I am super cool\r\n\r\n");
	}
	#endif

	// Basic Communication Testing
	while(1) {
		if (rxRadioFlag) {
			wireSend(received);
			rxRadioFlag = 0;
		}
		if (rxWireFlag) {
			radioSend(received);
			wireSend(received);
			rxWireFlag = 0;
		}

	}

	// Button testing
	while(0) {
		if (BUTTON_PRESSED)
			processButton();
	}
	
	// USART RX Testing
	while(0) {
		if (rxWireFlag) {
			sprintf(buf, "Pulse\r\n");
			radioSendString(buf);
			rxWireFlag = 0;
		}
	}

	// Switch Testing
	while(0) {
		//radioSend('R');
		radioSend('W');
		//radioSendString("Hello my name is Matt\r\n");
		//PORTD ^= 0xFF;
		//PORTC = getADC(0);
	}

	// LED Bar Testing
	while(0) {
		for (int i = 0; i < 256; i++) {
			//while (PIND & 0x01);
			PORTC = i;
			wait();
		}
	}

	// ADC testing
	while (0) {
	  	PORTC = getADC(0);
	  	wait();
	}

	void getData2() {
	data[0] = 'H';
	data[1] = 'e';
	data[2] = 'l';
	data[3] = 'l';
	data[4] = 'o';
	data[5] = '!';
 }

 // PROCESSBUTTON FUNCTION
// Does something when button is pressed, and debounces
void processButton() {	
	_delay_ms(10);
	//PORTC ^= 0x04;
	rcvrTmr = 1;
	while(BUTTON_PRESSED);
	_delay_ms(10);
}