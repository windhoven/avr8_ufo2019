/*
 * Ufo2019.cpp
 *
 * Created: 26-8-2019 14:24:13
 * Author : Johan
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/eeprom.h>

uint8_t EEMEM deviceConfig = {
	0x00 // Address
};


#define USART_BAUDRATE 9600
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)// 1mhz = 6, 8mhz = 51

#define BUFFER_SIZE 6

// The inputted commands are never going to be
// more than 8 chars long.
// volatile so the ISR can alter them
volatile static unsigned char rx_buffer[BUFFER_SIZE];
static unsigned char  command_in[BUFFER_SIZE];
volatile static unsigned char data_count;
volatile static unsigned char command_ready;

#ifndef TRUE
	#define TRUE 1
	#define FALSE 0
#endif

volatile static uint8_t ignore = TRUE;
volatile static uint16_t lastCmdCount =0;

#define N_LED 6

struct leds {
	uint8_t ledPort;
	int newValue;
	int currentValue;
	int waitValue;
	uint8_t mode; // 1 = Random, 0 = Manual
} ledData[] = {
	 {PB0, 100, 0, 0, 1}, // LED 1 RED
	 {PB1, 50, 0, 0, 1}, // LED 1 GREEN
	 {PB2, 25, 0, 0, 1}, // LED 1 BLUE
	 {PB3, 10, 0, 0, 1}, // LED 2 RED
	 {PB4, 25, 0, 0, 1}, // LED 2 GREEN
	 {PB5, 50, 0, 0, 1} // LED 2 BLUE
 };

volatile int pwm_phase = 0;
volatile int tel = 0;

uint8_t eAddress = 0;

void USART_Init(void)
{
	/* Set baud rate */
	UBRRH = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register
	UBRRL = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register
	
	/* Enable receiver and transmitter */
	UCSRB = (1<<RXEN)|(1<<RXCIE); //|(1<<TXEN);
	
	/* Set frame format: 8data, 2stop bit, no parity */
	UCSRC = (1 << UCSZ0) | (1 << UCSZ1); // Use 8-bit character sizes
}

// Reset to 0, ready to go again
void resetBuffer(void) {
	//for( int i = 0; i < BUFFER_SIZE;  ++i ) {
	//rx_buffer[i] = 0;
	//}
	rx_buffer[0] = 0;
	data_count = 0;
}

void processCommand(void) {
	// process command
	command_ready = FALSE;
	if (command_in[0] ==255 && command_in[1] > 0 && command_in[1] < 255) { // Programm new Address		
		eeprom_write_byte(&deviceConfig,command_in[1]);	
		eAddress = command_in[1];
	} else if (command_in[0] == eAddress) { // Set LED values
		uint8_t iLed = command_in[1];
		uint8_t iValue = command_in[2];
		uint8_t iMode = command_in[3];
		
		for (uint8_t i=0;i<N_LED;i++) {
			if (iLed && (1 << i)) {
				ledData[i].currentValue = iValue;
				ledData[i].mode = iMode;
			}
		}
	}
		
	ignore = FALSE;
}

void putCharToBuffer(unsigned char c) {
	if (command_ready == TRUE)  // commando still needs to be processed
	return;
	
	if (data_count >= BUFFER_SIZE) {
		// too much data
		resetBuffer();
		ignore = TRUE;
	}
	
	if (data_count == 0 && (c == eAddress || c == 255)) { // 255 = Address for programming new Address
		// wrong address
		ignore = FALSE;
	}

	rx_buffer[data_count++] = c;
	
	if (c == '\n') { // End of line!
		if (ignore == FALSE) {
			command_ready = TRUE;
			// copy command, so we can reset the buffer
			for( int i = 0; i < BUFFER_SIZE;  ++i ) {
				command_in[i] = rx_buffer[i];
			}
			// process command
			processCommand();
		}
		ignore = TRUE;
		resetBuffer();
		lastCmdCount = 8192;
	}
}

/*
 * ISR RX complete
 * Receives a char from UART and stores it in ring buffer.
 */
ISR(USART_RX_vect) {
	//	unsigned char value;
	//	value = UDR;  // Fetch the received byte value into the variable "value"
	//	UDR = value;    //Put the value to UDR = send

	// Get data from the USART in register
	unsigned char temp = UDR;
	
	putCharToBuffer(temp);	
}


uint8_t myRandomValue(uint8_t ibase, uint8_t irand) {
	return ibase +(rand() / (RAND_MAX / irand + 1));
}

void doPWMLed(leds *led) {
	if( led->currentValue == pwm_phase )
	{
		PORTB |= (1<< led->ledPort); // active low LED uit
	}
	if( pwm_phase == 100 )
	{
		PORTB &= ~(1<< led->ledPort); // active low LED aan
	}
}

void iUpDownCalc(leds *led) {
	if (led->mode == 1) {
		if (led->waitValue == 0) {
			if (led->newValue > led->currentValue)	{
				led->currentValue = led->currentValue+1;
			} else if (led->newValue < led->currentValue)	{
				led->currentValue = led->currentValue-1;
			} else {
				led->newValue = myRandomValue(0,10)*10; // 10 stappen voor pwm
				led->waitValue = myRandomValue(5,75);
			}
		} else {
			led->waitValue = led->waitValue -1;
		}
	}
}

ISR(TIMER0_OVF_vect)
{	
	if (++tel == 3) {
		PORTB ^= (1 << PB6) | (1 << PB7); // Toggle the LEDs
		//TCNT0 += 6; // Adjust count so we get 250 uS rollover period
		tel = 0;
	}
	if (tel == 0) {
		iUpDownCalc(&ledData[0]);
		iUpDownCalc(&ledData[1]);
	}  else if (tel == 1) {
		iUpDownCalc(&ledData[2]);
		iUpDownCalc(&ledData[3]);
	} else if (tel == 2) {
		iUpDownCalc(&ledData[4]);
		iUpDownCalc(&ledData[5]);
	}
	if (lastCmdCount > 0) {
		--lastCmdCount;
	}
}

/*
  Read random seed from eeprom and write a new random one.
*/
void initrand()
{
        uint32_t state;
        static uint32_t EEMEM sstate = 1;

        state = eeprom_read_dword(&sstate);

        // Check if it's unwritten EEPROM (first time). Use something funny
        // in that case.
        if (state == 0xffffffUL)
                state = 0xDEADBEEFUL;
        srand(state);
		
		state = !state;
        eeprom_write_dword(&sstate,rand());				
} 

int main(void)
{
	DDRB = (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3) | (1<<DDB4) | (1<<DDB5) | (1<<DDB6) | (1<<DDB7)  ; //(1<<DDB3); // = output
	PORTB = 0x00;
	PORTB |= ~(1<<PB6);
	
	// Setup Timer 0
	TCCR0A = 0b00000000;   // Normal Mode
	TCCR0B =  (1<<CS00) | (1<<CS02);   // Div 1024 Prescaler
	TCNT0 = 0;            // Initial value

	// Enable interrupts as needed
	TIMSK |= (1<<TOIE0); //(1<<OCIE0A);      // Timer 0 Interrupt
	
	USART_Init();
	 	
	eAddress = eeprom_read_byte(&deviceConfig);
	 		
	sei();               // Global Interrupts
	
	initrand();
	
    /* Replace with your application code */
    while (1) 
    {
		// begin pwm leds
		for (uint8_t i=0;i<N_LED;i++) {
			doPWMLed(&ledData[i]);
		}
		if( ++pwm_phase > 100 ) {
			pwm_phase = 0;
		}
		// end pwm leds
    }
}

