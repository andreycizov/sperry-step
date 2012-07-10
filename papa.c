#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/atomic.h>


#ifndef F_CPU
//define CPU clock speed if not defined
#define F_CPU 11059200
#endif

#include "degree.h"
#include "motor.h"
#include "nmea.h"
#include "usart.h"
#include "delay.h"

#define STEPS_MUL 2

#define STEP_TIMER_PRESCALER 64
#define STEP_TIMER_DEGR_PER_SECOND 3

#define MSG_TIMER_PRESCALER 1024
#define MSG_TIMER_SECONDS 3
#define MSG_TIMER_CTC F_CPU/MSG_TIMER_PRESCALER
#define MSG_TIMER_BIT 128

/* defines for msg ignore state */
#define MSG_IGNORE_PORT (PINB & 1)
const char MSG_IGNORE_STATE_STR[] = "0.0";
degree MSG_IGNORE_STATE_DEGREE = {0,0,10};

#define MSG_BUFFER_SIZE (4)
#define MSG_CURRENT_STEP_STR_SIZE (8)

int msg_timer_ctr = 0;
int msg_timer_max = 0;
// no message received flag
int msg_timer_flag = 0;

uint16_t msg_timer_out_ctr = 0;
// defaults to message per one second
uint16_t msg_timer_out_max = 1;

// we did not receive any messages before
int global_degr_first = 1;

// we're in the state of ignoring input
int global_msg_ignore = 0;

// current step in the motor epsilons
int32_t global_current_step;
// current step as as string length (optimization)
int global_current_step_str_size = 0;
// current step buffer
uint8_t global_current_step_str[MSG_CURRENT_STEP_STR_SIZE];

void init() {
	DDRA = 0x00;
	DDRB = 0x00;
	DDRC = 0xFF;
                PORTA=255;
	PORTB=254;
                _delay_loop_2 (50);  // delay 50 mc


	uint32_t stepnum = PINA & 7;
	uint32_t baudrate = (PINA >> 3) & 3;
	nmea_require_checksum = (PINA >> 5) & 1;

	msg_timer_out_max = ((PINA >> 6) & 1)? 1 : 10;
	
	// stepnum = number of steps per degree * 2
	switch(stepnum) {
	    case 0: stepnum  = 800; break; // 400 
		case 1: stepnum  =   2; break; //   1
		case 2: stepnum  =   3; break; //   1.5
		case 3: stepnum  =   8; break; //   4
		case 4: stepnum  =  12; break; //   6
		case 5: stepnum  =  20; break; //  10
		case 6: stepnum  =  48; break; //  24
		case 7: stepnum  =  96; break; //  48
		default: stepnum =  12; break; //   6 default
	}
	steps_per_degr = stepnum;
	steps_per_circle = stepnum * DEGR_MAX >> 1;
	
	switch(baudrate) {
		case 1: baudrate =  4800; break;
		case 2: baudrate =  9600; break;
		case 3: baudrate = 38400; break;
		case 0: baudrate =  4800; break;
	}

	usart_init(baudrate);
	motor_init((F_CPU*STEPS_MUL)/STEP_TIMER_PRESCALER/stepnum/STEP_TIMER_DEGR_PER_SECOND);
	msg_timer_init(F_CPU/MSG_TIMER_PRESCALER);

	// Enable no-message flag;
	PORTC |= MSG_TIMER_BIT;

	wdt_reset(); 
	wdt_enable(WDTO_60MS);
	
	sei(); //  Enable global interrupts
	set_sleep_mode(SLEEP_MODE_IDLE);
}

ISR(TIMER0_COMP_vect)
{
	msg_timer_ctr++;
	msg_timer_out_ctr++;
	if(msg_timer_out_ctr > msg_timer_out_max) {
		msg_timer_out_ctr = 0;
		if(msg_timer_flag && 1 != global_degr_first) {
			// emit a message
			nmea_msg_forward_heading(global_current_step_str, 
				global_current_step_str_size);
		}
	}

	if(msg_timer_ctr > msg_timer_max) {
		PORTC &= (~MSG_TIMER_BIT);
		msg_timer_ctr = 0;
		msg_timer_flag = 0;
	}
}

void msg_timer_init(uint32_t ocr0) {
	msg_timer_max = ocr0 * MSG_TIMER_SECONDS / 255;
	msg_timer_out_max = (uint16_t)(((uint32_t)((uint32_t)ocr0 * msg_timer_out_max) / (uint32_t)2550));
	OCR0 = 255;
	TIMSK |= (1 << OCIE0);
	TCCR0 |= ((1 << CS00) | (0 << CS01) | (1 << CS02)); // Start timer at Fcpu/64
}




void global_degr_update(degree next) {
	int32_t next_steps = degr_to_step(next);

	if(!global_degr_first) {
		int32_t diff = next_steps - global_current_step;
		
		int32_t diff_steps_one = 0;
		if(diff >= 0) {
			diff_steps_one = diff;
			 	
		} else {
			diff_steps_one = steps_per_circle + diff;
		}
		
		int32_t diff_steps_two = steps_per_circle - diff_steps_one;		

		int32_t diff_steps = (diff_steps_one < diff_steps_two ? diff_steps_one : - diff_steps_two);

		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{	
			motor_dir += diff_steps;

			if(motor_dir > (int32_t)steps_per_circle) {
				motor_dir -= steps_per_circle;
			} else if(motor_dir < -(int32_t)steps_per_circle) {
				motor_dir += steps_per_circle;
			}

			// Reset the flag.
			PORTC |= MSG_TIMER_BIT;
			msg_timer_ctr = 0;
			msg_timer_flag = 1;
		}
	} else {
		global_degr_first = 0;
	}

	global_current_step = next_steps;
}

void global_degr_str_update(uint8_t *field, int length) {
	// cut off the unneeded precision
	int i = 0;
	for(; i < length && i < MSG_CURRENT_STEP_STR_SIZE; i++) {
		global_current_step_str[i] = field[i];
	}
	global_current_step_str_size = i;
}

uint8_t msg_buffer[MSG_BUFFER_SIZE];

int main(void)
{
	init();
	
	while(1){	
		int l = usart_read(msg_buffer, MSG_BUFFER_SIZE);
		if(l > 0 && 0 == global_msg_ignore) {
			nmea_read(msg_buffer, l);
		}

		if(global_msg_ignore != MSG_IGNORE_PORT) {
			global_msg_ignore = MSG_IGNORE_PORT;
			if(1 == global_msg_ignore) {
				global_degr_update(MSG_IGNORE_STATE_DEGREE);
				global_degr_str_update(MSG_IGNORE_STATE_STR, sizeof(MSG_IGNORE_STATE_STR));
			}
		}
		wdt_reset(); 
	}
	return 0;
 }










