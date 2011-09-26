#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

#include "degree.h"
#include "nmea.h"
int nmea_msg_atoi_size(uint8_t *d, int count) {
	int size = 0;
	for(int i = 0;
	  d[i] != NMEA_SYMBOL_POINT_SEP &&
	  NMEA_SYMBOL_IS_DECIMAL(d[i]) &&
	  i < count; i++){
		size++;
	}
	return size;
}

uint32_t nmea_msg_atoi(uint8_t *d, int count) {
	uint32_t mul = 10;
	uint32_t r = 0;
	for(int i = count - 1; i >= 0; i-- ) {
		r += (d[i] - NMEA_SYMBOL_0) * mul;
		mul *= 10;
	}
	return r;
}

int nmea_msg_ansi_to_degr(uint8_t *d, int count, degree *r) {
	if(count == 0)
		return -1;
	int size_i = nmea_msg_atoi_size(d, count);

	if(size_i == 0)
		return -1;

	r->i = nmea_msg_atoi(d, size_i);
	r->frac = 0;
	r->denom = 1;

	count -= size_i;

	if(count == 0)
		return size_i;

	if(d[0] != NMEA_SYMBOL_POINT_SEP)
		return -1;

	d++;
	count -= 1;

	int size_frac = nmea_msg_atoi_size(d, count);

	r->frac = nmea_msg_atoi(d, size_frac);
	r->denom = size_frac > 0?size_frac:1;

	return size_i + size_frac;
}

#include "usart.h"

#include "motor.h"


#ifndef F_CPU
//define CPU clock speed if not defined
#define F_CPU 4000000
#endif

#define STEPS_MUL 2

#define STEP_TIMER_PRESCALER 64
#define STEP_TIMER_DEGR_PER_SECOND 5

#define MSG_TIMER_PRESACLER 1024
#define MSG_TIMER_SECONDS 2
#define MSG_TIMER_CTC F_CPU/MSG_TIMER_PRESCALER/MSG_TIMER_SECONDS
#define MSG_TIMER_CTC2 16

void init() {
	DDRA = 0x00;
	DDRB = 0x00;
	DDRC = 0xFF;
	uint32_t stepnum = PORTA & 7;
	uint32_t baudrate = (PORTA >> 3) & 3;
	
	// stepnum = number of steps per degree * 2
	switch(stepnum) {
		case 1: stepnum  =   2; break; //   1
		case 2: stepnum  =   3; break; //   1.5
		case 3: stepnum  =   8; break; //   4
		case 4: stepnum  =  12; break; //   6
		case 5: stepnum  =  20; break; //  10
		case 6: stepnum  = 384; break; // 192
		case 7: stepnum  =  96; break; //  48
		default: stepnum =  12; break; //   6 default
	}
	steps_per_degr = stepnum;
	steps_per_circle = stepnum * DEGR_MAX;
	
	switch(baudrate) {
		case 1: baudrate =  4800; break;
		case 2: baudrate =  9600; break;
		case 3: baudrate = 38400; break;
		case 0: baudrate =  4800; break;
	}

	usart_init(baudrate);
	motor_init((F_CPU*STEPS_MUL)/STEP_TIMER_PRESCALER/stepnum/STEP_TIMER_DEGR_PER_SECOND);

	sei(); //  Enable global interrupts
	set_sleep_mode(SLEEP_MODE_IDLE);
}




#define MSG_BUFFER_SIZE (5)
uint8_t msg_buffer[MSG_BUFFER_SIZE];

int main(void)
{
	init();
	
	degree a = {255,10,100},
	b = {150,10,100},
	d = {1, 4,10},c, e,f,g;
	degr_sub(a,b, &c);
	degr_sub(a,d,&e);
	degr_sub(d,a,&f);
	degr_sub(b,a,&g);
	uint32_t st = degr_to_step(d);
	
	//PORTC = 0xff;
	while(1){	
		int l = usart_read(msg_buffer, MSG_BUFFER_SIZE);
	   //usart_read_block(msg_buffer, MSG_BUFFER_SIZE);
//	   TCNT1 = 0;
		if(l > 0) {
			nmea_read(msg_buffer, l);
		}
	}
	return 0;
 }





