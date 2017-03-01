/** original version: PPMDECODER.c

*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

#ifndef F_CPU
#define F_CPU 12000000UL
#endif

#define MIN_SYNC_LEN 4000L
#define MIN_PULSE_LEN 800L
#define MAX_PULSE_LEN 2200L
#define NEUTRAL_PULSE_LEN 1500L
#define NEG_PULSE_LEN 200L
#define NORM_FRAME_LEN 22500L
#define SYNC_SPACER NORM_FRAME_LEN/5
#define MIN_FRAME_LEN NORM_FRAME_LEN-SYNC_SPACER
#define MAX_FRAME_LEN NORM_FRAME_LEN+SYNC_SPACER

#define NUM_PULSE_PER_FRAME 5
#define MIN_NUM_VALID_FRAMES 2 // number of valid frames required to switch back from NORC
#define MAX_NUM_INVALID_FRAMES 4 // number of invalid frames required to switch to NORC

#define STATE_NORC 0 // no valid ppm, use neutral value
#define STATE_MANUAL 1 // valid ppm

#define MAX_TWI_MSG_LEN 21
#define

#define TWI_ACK _BV(TWINT)|_BV(TWEA)|_BV(TWIE)|_BV(TWEN) // TWINT=Interrupt flag,TWEA=Enable acknowledge,TWIE=Interrupt enable,TWEN=TWI enable

uint16_t inPW[NUM_PULSE_PER_FRAME]; // servo values from R/C receiver
uint16_t digitalData[NUM_PULSE_PER_FRAME]; // 5 bits control data
uint16_t neutralPW[NUM_PULSE_PER_FRAME] = NEUTRAL_PULSE_LEN; // neutral servo values
uint16_t stat[NUM_PULSE_PER_FRAME];

volatile uint8_t CurrentState = STATE_NORC;

volatile uint8_t i2cIn[MAX_TWI_MSG_LEN];
valatile uint8_t i2cFinished = 0;

uint16_t DecimaltoBinary(const uint8_t Decnumber)
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
	
	uint8_t invalidFrameCount = 0;
	uint8_t validFrameCount = 0;
	
	// Power and noise reduction
	PRR = _BV(PRTIM2)|_BV(PRTIM0)|_BV(PRSPI)|_BV(PRUSART0)|_BV(PRADC);
	
	i2cIn[0] = 0x80;
	
	DDRB = 0b00000010;
	
	ICR1 = 0;
	
	TCCR1B = (0 << ICES1)|_BV(CS11); //prescaler 8, input capture according to PPM negative edge
	
	TWCR = _BV(TWEN)|_BV(TWIE)|_BV(TWINT)|_BV(TWEA)|_BV(TWSTA)|_BV(TWSTO)|_BV(TWWC);
	
	sei();
	
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
				if (validSync && (inFrameLen > MIN_FRAME_LEN) && (inFrameLen < MAX_FRAME_LEN) && (inPulseCount == NUM_PULSE_PER_FRAME)) {
					invalidFrameCount = 0;
					if (validFrameCount < MIN_NUM_VALID_FRAMES) validFrameCount++;
				} else {
					validFrameCount = 0;
					if (invalidFrameCount < MAX_NUM_INVALID_FRAMES) invalidFrameCount++;
				}
				validSync = 1;
				inPulseCount = 0;
			} else if (validSync && ValidPulseLen(PulseLen) && (inPulseCount < NUM_PULSE_PER_FRAME)) {
				inPW[inPulseCount++] = PulseLen;
			} else {
				validSync = 0;
			}
		}
		
		for (i=0; i<=NUM_PULSE_PER_FRAME;i++) {
			if ((800 < inPW[i]) && (inPW[i] <= 844)) digitalData[i] = DecimaltoBinary(0);
			else if ((845 <= inPW[i]) && (inPW[i] <= 888)) digitalData[i] = DecimaltoBinary(1);
			else if ((889 <= inPW[i]) && (inPW[i] <= 932)) digitalData[i] = DecimaltoBinary(2);
			else if ((933 <= inPW[i]) && (inPW[i] <= 976)) digitalData[i] = DecimaltoBinary(3);
			else if ((977 <= inPW[i]) && (inPW[i] <= 1020)) digitalData[i] = DecimaltoBinary(4);
			else if ((1021 <= inPW[i]) && (inPW[i] <= 1064)) digitalData[i] = DecimaltoBinary(5);
			else if ((1065 <= inPW[i]) && (inPW[i] <= 1108)) digitalData[i] = DecimaltoBinary(6);
			else if ((1109 <= inPW[i]) && (inPW[i] <= 1152)) digitalData[i] = DecimaltoBinary(7);
			else if ((1153 <= inPW[i]) && (inPW[i] <= 1196)) digitalData[i] = DecimaltoBinary(8);
			else if ((1197 <= inPW[i]) && (inPW[i] <= 1240)) digitalData[i] = DecimaltoBinary(9);
			else if ((1241 <= inPW[i]) && (inPW[i] <= 1184)) digitalData[i] = DecimaltoBinary(10);
			else if ((1185 <= inPW[i]) && (inPW[i] <= 1328)) digitalData[i] = DecimaltoBinary(11);
			else if ((1329 <= inPW[i]) && (inPW[i] <= 1372)) digitalData[i] = DecimaltoBinary(12);
			else if ((1373 <= inPW[i]) && (inPW[i] <= 1416)) digitalData[i] = DecimaltoBinary(13);
			else if ((1417 <= inPW[i]) && (inPW[i] <= 1460)) digitalData[i] = DecimaltoBinary(14);
			else if ((1461 <= inPW[i]) && (inPW[i] <= 1504)) digitalData[i] = DecimaltoBinary(15);
			else if ((1505 <= inPW[i]) && (inPW[i] <= 1548)) digitalData[i] = DecimaltoBinary(16);
			else if ((1549 <= inPW[i]) && (inPW[i] <= 1592)) digitalData[i] = DecimaltoBinary(17);
			else if ((1593 <= inPW[i]) && (inPW[i] <= 1636)) digitalData[i] = DecimaltoBinary(18);
			else if ((1637 <= inPW[i]) && (inPW[i] <= 1680)) digitalData[i] = DecimaltoBinary(19);
			else if ((1681 <= inPW[i]) && (inPW[i] <= 1724)) digitalData[i] = DecimaltoBinary(20);
			else if ((1725 <= inPW[i]) && (inPW[i] <= 1768)) digitalData[i] = DecimaltoBinary(21);
			else if ((1769 <= inPW[i]) && (inPW[i] <= 1812)) digitalData[i] = DecimaltoBinary(22);
			else if ((1813 <= inPW[i]) && (inPW[i] <= 1856)) digitalData[i] = DecimaltoBinary(23);
			else if ((1857 <= inPW[i]) && (inPW[i] <= 1900)) digitalData[i] = DecimaltoBinary(24);
			else if ((1901 <= inPW[i]) && (inPW[i] <= 1944)) digitalData[i] = DecimaltoBinary(25);
			else if ((1945 <= inPW[i]) && (inPW[i] <= 1988)) digitalData[i] = DecimaltoBinary(26);
			else if ((1989 <= inPW[i]) && (inPW[i] <= 2032)) digitalData[i] = DecimaltoBinary(27);
			else if ((2033 <= inPW[i]) && (inPW[i] <= 2076)) digitalData[i] = DecimaltoBinary(28);
			else if ((2077 <= inPW[i]) && (inPW[i] <= 2120)) digitalData[i] = DecimaltoBinary(29);
			else if ((2121 <= inPW[i]) && (inPW[i] <= 2164)) digitalData[i] = DecimaltoBinary(30);
			else if ((2165 <= inPW[i]) && (inPW[i] <= 2200)) digitalData[i] = DecimaltoBinary(31);
		} // convert into 5 bits data
	}
}		
			
		
		
		
		
		    
		
			
				
	
	
