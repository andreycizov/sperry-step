#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include "usart.h"
//#include "nmea.h"
#ifndef F_CPU
//define cpu clock speed if not defined
#define F_CPU 4000000
#endif


#define NMEA_INPUT_BUFFER_SIZE 128

#define NMEA_SYMBOL_START '$'
#define NMEA_SYMBOL_CHECKSUM '*'
#define NMEA_SYMBOL_END 0x0A

uint8_t nmea_input_buffer[NMEA_INPUT_BUFFER_SIZE];
int nmea_input_buffer_i = 0;
int nmea_received_messages = 0;

int16_t nmea_msg_hex_to_byte(uint8_t *d) {
	uint8_t r = 0;
	for(int
	 i = 4; i >= 0; i-=4 ) {
		if(*d >= 0x30 && *d <= 0x39) {
			r+= ((*d - 0x30) << i);
		} else if (*d >= 0x41 && *d <=0x46) {
			r+= ((*d - 0x41) << i);
		} else {
			return -1;
		}
		d++;
	}
	return (int16_t)r;
}

uint8_t nmea_checksum(uint8_t *d, int length) {
	uint8_t cs = 0;
	while(length > 0) {
		cs ^= *d;
		d++;
		length--;
	}
	return cs;
}

int nmea_msg_symbol(uint8_t symbol, uint8_t *d, int count) {
	for(int i = 0; i < count; i++) {
		if(d[i] == symbol)
			return i;
	}
	return -1;
}

int nmea_process(uint8_t *d, int count) {
	uint8_t *orig_d = d;
	while( 1 ) {
		int end = nmea_msg_symbol(NMEA_SYMBOL_END, d, count);
		if( end == -1) {
			if((d - orig_d) == 0){
				return -1;
			}  else {
				return d - orig_d;
			}
		}
		int start = nmea_msg_symbol(NMEA_SYMBOL_START, d, end);
		while(1) {
			if( start == -1 || end - start < 10 )
				break;
			//@TODO check start and end difference here!! check for <CR> symbol here
			// $XXXYY*CS<CR><LF>
			// 0123456789	0
			d += start + 1;
			end -= start + 1;
			
			int checksum = nmea_msg_symbol(NMEA_SYMBOL_CHECKSUM, d, end);

			// Difference between checksum and message end is 4 bytes.
			if( checksum == -1 || end - checksum != 4 )
				break;
			uint16_t cs1 = nmea_checksum(d, checksum);	
			uint16_t cs2 = nmea_msg_hex_to_byte(d+checksum+1);
			if( ((int16_t)nmea_checksum(d, checksum)) != nmea_msg_hex_to_byte(d+checksum+1) )
				break;
			nmea_received_messages++;	
			//message processed here!!!
			
			break;
		}
		d += end + 1;
		count -= end + 1;
	}
}

void nmea_copy(uint8_t *d, int count) {
	int left = NMEA_INPUT_BUFFER_SIZE - nmea_input_buffer_i;
	count = count > left ? left:count;
	memcpy(nmea_input_buffer + nmea_input_buffer_i, d, count);
	nmea_input_buffer_i += count;
}

// reads n bytes from buffer
void nmea_read(uint8_t *d, int count) {
	nmea_copy(d, count);
	
	int end = nmea_process(nmea_input_buffer, nmea_input_buffer_i);
	PORTB = end;			
	if((end == -1) && ((nmea_input_buffer_i + 1) == NMEA_INPUT_BUFFER_SIZE)) {
		nmea_input_buffer_i = 0;
		return;
	} else if (end >= 0) {
		int t = nmea_input_buffer_i - end;
		nmea_input_buffer_i = 0;
		nmea_copy(nmea_input_buffer + end, NMEA_INPUT_BUFFER_SIZE - end - 1);
		nmea_input_buffer_i = t;	
	}
}
#define DEGR_MAX 360

uint32_t steps_per_degr = 0;
uint32_t steps_per_circle = 0;

void init() {
	DDRA = 0x00;
	DDRB = 0x00;
	DDRC = 0xFF;
	uint32_t stepnum = PORTA & 7;
	uint32_t baudrate = (PORTA & 24) >> 3;
	
	// stepnum = number of steps per degree * 2
	switch(stepnum) {
		case 1: stepnum  =   2; break;
		case 2: stepnum  =   3; break;
		case 3: stepnum  =   8; break;
		case 4: stepnum  =  12; break;
		case 5: stepnum  =  20; break;
		case 6: stepnum  = 384; break;
		default: stepnum =  12; break; // TODO: default values
	}
	stepnum = 384;
	//@TODO delete
	steps_per_degr = stepnum;
	steps_per_circle = stepnum * DEGR_MAX;
	
	switch(baudrate) {
		case 1: baudrate =  4800; break;
		case 2: baudrate =  9600; break;
		case 3: baudrate = 38400; break;
		case 0: baudrate =  4800; break;
	}
	usart_init(baudrate);
	
	TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode

   TIMSK |= (1 << OCIE1A); // Enable CTC interrupt

	// @TODO: timer is proportional, max 5degrees/sec
   OCR1A   = 600; // Set CTC compare value to 1Hz at 1MHz AVR clock, with a prescaler of 64

   TCCR1B |= ((1 << CS10) | (1 << CS11) | (0 << CS12)); // Start timer at Fcpu/64

	sei(); //  Enable global interrupts
	set_sleep_mode(SLEEP_MODE_IDLE);
}
// this thing should be carefully used in other parts of code!!!
int32_t motor_dir = 0;
#define MOTOR_NUM 3
#define MOTOR_STATE_MASK (7)
#define MOTOR_STATE_MAX (MOTOR_NUM*2-1)
#define MOTOR_PORT PORTC
int motor_state = 0;

void motor_next() {
	if(motor_dir == 0)
		return;
		
	int dir = (motor_dir > 0) ? 1 : -1;
	motor_dir -= dir;
	motor_state += dir;
	
	if(motor_state > MOTOR_STATE_MAX) {
		motor_state = 0;
	} else if(motor_state < 0) {
		motor_state = MOTOR_STATE_MAX;
	}
	
	//count each halfstep as odd motor state val
	uint8_t step_mask = 2 | (motor_state & 1);
	// motor_state / 2
	uint8_t step_full = motor_state >> 1;
	// rotate the mask based on our current state
	uint8_t motor_mask = ((step_mask >> step_full) |
		(step_mask << (MOTOR_NUM - step_full)))
		& MOTOR_STATE_MASK;
	MOTOR_PORT = (MOTOR_PORT & (~MOTOR_STATE_MASK)) | motor_mask;
}

ISR(TIMER1_COMPA_vect)
{
    motor_next();
}

typedef struct degree {
	int16_t i;
	int8_t frac;
	int8_t denom;
} degree;

void degr_sub(degree n1, degree n2, degree *r) {
	if(n1.denom < n2.denom) {
		n1.frac = n2.denom / n1.denom * n1.frac;
		r->denom = n2.denom;
	} else if (n2.denom < n1.denom) {
		n2.frac = n1.denom / n2.denom * n2.frac;
		r->denom = n1.denom;
	} else {
		r->denom = n1.denom;
	}
	
	r->frac = n1.frac - n2.frac;
	r->i = 0;
	
	// Normalise values
	if(r->frac < 0) {
		r->frac = r->denom + r->frac;
		r->i--;
	}
	r->i = n1.i - n2.i;
	if(r->i < 0)
		r->i = DEGR_MAX + r->i;
}

uint32_t degr_to_step(degree d) {
	uint32_t steps_i = d.i * steps_per_degr;
	uint32_t steps_f = (d.frac * steps_per_degr) / d.denom;
	return (steps_i + steps_f) >> 1;
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





