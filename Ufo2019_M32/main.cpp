/*
 * Ufo2019_M32.cpp
 *
 * Created: 4-9-2019 9:11:35
 * Author : Ch4oZ
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include "LightweightRingBuff.h"
#include <stdbool.h>

uint8_t EEMEM deviceConfig = {
	0x0B // Address
};

#define FOSC F_CPU // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

#define CMD_BUFFER_SIZE 12

RingBuff_t Buffer;

#define N_LED 6

struct leds {
	uint8_t ledPin;
	uint8_t newValue;
	uint8_t currentValue;
	uint8_t waitValue;
	uint8_t ledMode; // 1 = Random, 2 = Manual
} volatile ledData[] = {
	 {PINB0, 0, 100, 0, 1}, // LED 1 RED  // 0= 100% ON , 100 = OFF
	 {PINB1, 0, 100, 0, 1}, // LED 1 GREEN
	 {PINB2, 0, 100, 0, 1}, // LED 1 BLUE
	 {PIND5, 0, 100, 0, 1}, // LED 2 RED
	 {PIND6, 0, 100, 0, 1}, // LED 2 GREEN
	 {PIND7, 0, 100, 0, 1} // LED 2 BLUE
 };

// The inputted commands are never going to be
// more than 8 chars long.
// volatile so the ISR can alter them
unsigned char command_in[CMD_BUFFER_SIZE];
uint8_t data_count; // doesn't need volatile ?
unsigned char command_ready;
uint8_t ignore;
uint16_t lastCmdCount =0;


volatile uint8_t pwm_phase = 0;
volatile uint8_t tel = 0;

unsigned char eAddress = 0;

void USART_Init(unsigned int ubrr)
{
	/* Set baud rate */	
	UBRR0H = (unsigned char)(ubrr>>8);	// Load upper 8-bits of the baud rate value into the high byte of the UBRR register
	UBRR0L = (unsigned char)ubrr;		// Load lower 8-bits of the baud rate value into the low byte of the UBRR register
	
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<RXCIE0); //|(1<<TXEN);
	
	/* Set frame format: 8data, 2stop bit, no parity */
	UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes
	
	data_count = 0;
	command_ready = false;
	ignore = true;
}

// Reset to 0, ready to go again
/*
void resetBuffer(void) {
	//for( int i = 0; i < CMD_BUFFER_SIZE;  ++i ) {
	//rx_buffer[i] = 0;
	//}
	command_in[0] = 0;
	data_count = 0;
}*/

/*
 * ISR RX complete
 * Receives a char from UART and stores it in ring buffer.
 */
ISR(USART_RX_vect) {
	//	unsigned char value;
	//	value = UDR;  // Fetch the received byte value into the variable "value"
	//	UDR = value;    //Put the value to UDR = send

	// Get data from the USART in register
	unsigned char temp = UDR0;
	
	if (RingBuffer_IsFull(&Buffer) == false) {
		RingBuffer_Insert(&Buffer, temp);
	}
}


uint8_t myRandomValue(uint8_t ibase, uint8_t irand) {
	return ibase +(rand() / (RAND_MAX / irand + 1));
}

void iUpDownCalc(uint8_t ix) {
	if (command_ready == false && ledData[ix].ledMode == 1) { // if command_ready == true then these values will be manipulated elsewhere.
		if (ledData[ix].waitValue == 0) {
			if (ledData[ix].newValue > ledData[ix].currentValue)	{
				ledData[ix].currentValue++;
			} else if (ledData[ix].newValue < ledData[ix].currentValue)	{
				ledData[ix].currentValue--;
			} else {
				ledData[ix].newValue = myRandomValue(0,10)*10; // 10 steps for pwm
				ledData[ix].waitValue = myRandomValue(5,75);
			}
		} else {
			ledData[ix].waitValue--;
		}
	}
}

ISR(TIMER2_OVF_vect)
{
	// begin pwm leds
	for (uint8_t i=0;i<N_LED;i++) {
		if( pwm_phase == 100)
		{
			if (ledData[i].ledPin >= 5 && ledData[i].ledPin <= 7) {
				PORTD &= ~(1<< ledData[i].ledPin); // LED uit
			} else {
				PORTB &= ~(1<< ledData[i].ledPin); // LED uit
			}
		}
		if( ledData[i].currentValue == pwm_phase && pwm_phase != 100)
		{
			if (ledData[i].ledPin >= 5 && ledData[i].ledPin <= 7) {
				PORTD |= (1<< ledData[i].ledPin); // LED aan
				} else {
				PORTB |= (1<< ledData[i].ledPin); // LED aan
			}
		}
	}
	if (pwm_phase == 100) {
		pwm_phase = 0;
	} else {
		pwm_phase++;
	}
	// end pwm leds
}

ISR(TIMER0_OVF_vect)
{			
	if (tel >=0 && tel < N_LED) {
		iUpDownCalc(tel);	// Fading from 'color' to 'color'
	}
	if (++tel == 9) {
		PORTB ^= (1 << PINB3) | (1 << PINB4); // Toggle the LEDs		
		tel = 0;
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
	DDRB |= (1<<DDB0) | (1<<DDB1) | (1<<DDB2) | (1<<DDB3) | (1<<DDB4);
	DDRD |= (1<<DDD5) | (1<<DDD6) | (1<<DDD7)  ; //(1<<DDB3); // = output
	PORTB &= ~((1<<PINB0) | (1<<PINB1) | (1<<PINB2) | (1<<PINB3) | (1<<PINB4));
	PORTD &= ~((1<<PIND5) | (1<<PIND6) | (1<<PIND7) ); //| (1<<PIND0)
	PORTB |= ~(1<<PINB3);
	
	eAddress = eeprom_read_byte(&deviceConfig);	
	
	initrand();	
	
	// Initialize the buffer with the created storage array
	RingBuffer_InitBuffer(&Buffer);
	
	for (uint8_t i=0;i<N_LED;i++) {
		ledData[i].newValue = myRandomValue(0,10)*10;
	}
	
	// Setup Timer 0
	
	TCCR0A = 0b00000000;   // Normal Mode
	TCCR0B =  (1<<CS00) | (1<<CS02);   // Div 1024 Prescaler
	TCNT0 = 0;            // Initial value
	
	// Enable interrupts as needed
	TIMSK0 |= (1<<TOIE0); //(1<<OCIE0A);      // Timer 0 Interrupt	
	
	// Setup Timer 2
	
	TCCR2A = 0b00000000;   // Normal Mode
	TCCR2B =  (1<<CS21) ;   // Div 1024 Prescaler
	TCNT2 = 0;            // Initial value

	// Enable interrupts as needed
	TIMSK2 |= (1<<TOIE2); //(1<<OCIE0A);      // Timer 2 Interrupt	
	
	USART_Init(MYUBRR);
	 		
	sei();               // Global Interrupts
	
	bool endOfCommand = false;
	
    /* Replace with your application code */
    while (1) 
    {				 
		 // Print contents of the buffer one character at a time		 
		 while (RingBuffer_IsEmpty(&Buffer) == false && command_ready == false) {
			unsigned char c = RingBuffer_Remove(&Buffer);							
			
			if (data_count >= CMD_BUFFER_SIZE) {
				// too much data
				endOfCommand = false;
				data_count = 0;
				ignore = true;
			}					
				
			if (data_count == 0 && c == eAddress) { // 255 = Address for programming new Address
				// wrong address
				ignore = false;
			}
			
			command_in[data_count++] = c;	
			
			if (c == '\r') { // End of line!		
				if (endOfCommand == true) {
					if (ignore == false) {
						command_ready = true;
						// process command
					} else {						
						data_count  =0;
						ignore = true;						
					}					
					lastCmdCount = 8192;					
				}				
			} else {
				endOfCommand = false;
			}
			if (c == '\n') { // End of line!
				endOfCommand = true;				
			}
		 }
		
		if (command_ready == true)	{				
			if (command_in[0] == eAddress && data_count >= 3) { // Set LED values								
				uint8_t iLeds = command_in[1];
				uint8_t iValue = command_in[2];
				uint8_t iPwm = iValue;
				if (iValue > 100) {
					iValue = 100;//(uint8_t)(iValue/2.55); //max value = 255 eq max pwm 100
				}		
				iPwm = 100-iValue; // 100 = OFF, 0 = 100% ON
				for (uint8_t ix=0;ix < N_LED;ix++) {
					if ( (iLeds & (1 << ix)) != 0) { // iFirst = LEDs
						ledData[ix].newValue = iPwm;
						if (data_count >= 4) {
							if (command_in[3] == 2) {
								ledData[ix].waitValue = 0;
								ledData[ix].ledMode = 2; // 0= manual
							} else {
								ledData[ix].ledMode = 1;
							}
						}
						if (ledData[ix].ledMode == 2) {
							ledData[ix].currentValue = iPwm;
						}
					}
				}
			}
			data_count =0;						
			command_ready = false;
			ignore = true;
		}
		
		if (lastCmdCount >0) {
			lastCmdCount--;
			if (lastCmdCount == 0) {
				for (uint8_t i=0;i<N_LED;i++) {
					ledData[i].ledMode = 1; // Do random value stuff
				}
			}
		}
    }
	return(0);
}