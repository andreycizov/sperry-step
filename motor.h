#define MOTOR_NUM 3

#define MOTOR_DRIVER_PINS (7)

#define MOTOR_STATE_MASK (7)
#define MOTOR_STATE_MAX (MOTOR_NUM*2-1)
#define MOTOR_PORT PORTC

#define MOTOR_2PHASE_STATE_MASK (3)

// this thing should be carefully used in other parts of code!!!
int32_t motor_dir = 1;
int motor_state = 0;

//default motor type to 3-phase motor
int motor_type = 0;

void motor_init(uint32_t ocr1a) {
	TCCR1B |= (1 << WGM12); // Configure timer 1 for CTC mode

	TIMSK |= (1 << OCIE1A); // Enable CTC interrupt

	OCR1A  = ocr1a;
	// @TODO: change this part based on the prescaler value in main.c
	TCCR1B |= ((1 << CS10) | (1 << CS11) | (0 << CS12)); // Start timer at Fcpu/64

	motor_type = PORTA & 128;

	MOTOR_PORT &= (~MOTOR_DRIVER_PINS);
}

void motor_next_3phase() {
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

uint8_t motor_states_2phase[] = {
	0b00,
	0b10,
	0b11,
	0b01
};

void motor_next_2phase() {
	uint8_t motor_mask = motor_states_2phase[(motor_state & MOTOR_2PHASE_STATE_MASK)];
	MOTOR_PORT = (MOTOR_PORT & (~MOTOR_2PHASE_STATE_MASK)) | motor_mask;
}

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

	if(motor_type == 0) {
		motor_next_3phase();
	} else {
		motor_next_2phase();
	}
}

ISR(TIMER1_COMPA_vect)
{
    motor_next();
}
