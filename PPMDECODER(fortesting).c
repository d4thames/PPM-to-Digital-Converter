/** Ideas on PPM decoder is from 
http://diydrones.com/profiles/blogs/decoding-multichannel-ppm
*/

#include <avr/io.h>
#include <util/delay.h>

#ifndef F_CPU
#define F_CPU 12000000UL
#endif
#define BDRATE_BAUD 9600

#define MIN_SYNC_LEN 4000L
#define MIN_PULSE_LEN 800L
#define MAX_PULSE_LEN 2200L
#define NEUTRAL_PULSE_LEN 1500L
#define NEG_PULSE_LEN 200L
#define NORM_FRAME_LEN 22500L
#define SYNC_SPACER NORM_FRAME_LEN/5
#define MIN_FRAME_LEN NORM_FRAME_LEN-SYNC_SPACER
#define MAX_FRAME_LEN NORM_FRAME_LEN+SYNC_SPACER

#define NUM_PULSE_PER_FRAME 1
#define MIN_NUM_VALID_FRAMES 2 // number of valid frames required to switch back from NORC
#define MAX_NUM_INVALID_FRAMES 4 // number of invalid frames required to switch to NORC


uint16_t inPW[NUM_PULSE_PER_FRAME] ; // servo values from R/C receiver
uint16_t digitalData[NUM_PULSE_PER_FRAME]; // 5 bits control data
uint16_t neutralPW[NUM_PULSE_PER_FRAME]; // neutral servo values AKA central positions
uint16_t stat[NUM_PULSE_PER_FRAME];

uint16_t DecimalToBinary(const uint8_t Decnumber)
{
	uint16_t Result = 0;
	uint8_t i = 1;
	uint8_t tmp = Decnumber;
	while (tmp > 0) {
		Result = (tmp % 2)*i;
		i = i*10;
		tmp = (tmp/2) - (tmp%2);
	}
	return Result;
}
void init_stdio2uart0(void)
{
	// configure UART0 baud rate, one start bit, 8-bit, no parity and one stop bit
	UBRR0H = (F_CPU/(BDRATE_BAUD*16L)-1) >> 8;
	UBRR0L = (F_CPU/(BDRATE_BAUD*16L)-1);
	UCSR0B = _BV(RXEN0)|_BV(TXEN0);
	UCSR0C = _BV(UCSZ00)|_BV(UCSZ01);
	
}
void binaryoutput(const uint16_t Outputnumber)
{
	DDRA = 0xFF;
	uint8_t remainer = 0;
	uint16_t tmp = Outputnumber;
	while (tmp > 1) {
		remainer = tmp % 10;
		tmp = tmp/10 - remainer;
		if (remainer == 1) { 
			PORTA |= _BV(PINA1);
			_delay_ms(1);
		} else {
			PORTA |= ~_BV(PINA1);
			_delay_ms(1);
		}
	}
}
void Transfer_numb(const uint16_t Controldata)
{
	// output the 5 bits data through UDR0
	while (!(UCSR0A & _BV(UDRE0)));
	UDR0 = Controldata;
}
static inline uint8_t ValidPulseLen (const uint16_t pulseLen)
{
	return (pulseLen > MIN_PULSE_LEN && pulseLen < MAX_PULSE_LEN);
}

int main(void)
{
	
	uint8_t i;
	uint16_t lastIcrTime = 0;
	uint8_t inPulseCount = 0;
	uint16_t inSyncTime = 0;
	uint16_t inFrameLen = 0;
	
	uint8_t validSync = 0;
	
	// Initiate the servo values with neutral values
	for (i=0;i< NUM_PULSE_PER_FRAME;i++) {
		inPW[i] = NEUTRAL_PULSE_LEN;
	}
	// Power and noise reduction
	PRR = _BV(PRTIM2)|_BV(PRTIM0)|_BV(PRSPI)|_BV(PRUSART0)|_BV(PRADC);
	
	DDRD = 0b01000000; //PD6 as input (ICP1)
	
	ICR1 = 0;
	
	TCCR1B = (0 << ICES1)|_BV(CS11); //prescaler 8, input capture according to PPM negative edge
	
	
	while (1) {
		if (TIFR1 & _BV(ICF1)) {
			uint16_t currIcrTime = ICR1;
			TIFR1 = _BV(ICF1);
			uint16_t PulseLen = currIcrTime - lastIcrTime;
			lastIcrTime = currIcrTime;
			if ((PulseLen > MIN_SYNC_LEN) && (PulseLen < NORM_FRAME_LEN)) {
				//sync detected
				inFrameLen = currIcrTime - inSyncTime;
				inSyncTime = currIcrTime;
				validSync = 1;
				inPulseCount = 0;
			} else if (validSync && ValidPulseLen(PulseLen) && (inPulseCount < NUM_PULSE_PER_FRAME)) {
				inPW[inPulseCount++] = PulseLen;
			} else {
				validSync = 0;
			}
		}
		
		for (i=0; i<NUM_PULSE_PER_FRAME;i++) {
			digitalData[i] = DecimalToBinary((inPW[i]-800)/44);
		} // convert into 5 bits data
		for (i=0; i<NUM_PULSE_PER_FRAME;i++) {
			Transfer_numb(digitalData[i]);
		} // sending 5-bit binary through UDR0
		for (i=0; i<NUM_PULSE_PER_FRAME;i++) {
			binaryoutput(digitalData[i]);
		}
		
	}
}

			
		
		
		
		
		    
		
			
				
	
	
