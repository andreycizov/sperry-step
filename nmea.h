#include "degree.h"

#define NMEA_INPUT_BUFFER_SIZE 128

#define NMEA_SYMBOL_START '$'
#define NMEA_SYMBOL_CHECKSUM '*'
#define NMEA_SYMBOL_END 0x0A

#define NMEA_SYMBOL_FIELD_SEP ','
#define NMEA_SYMBOL_POINT_SEP '.'

#define NMEA_SYMBOL_0 0x30
#define NMEA_SYMBOL_9 0x39
#define NMEA_SYMBOL_A 0x41
#define NMEA_SYMBOL_F 0x46
#define NMEA_SYMBOL_IS_DECIMAL(s) (s >= NMEA_SYMBOL_0 && s <= NMEA_SYMBOL_9)

#define NMEA_MSG_HDR_SIZE 3
const char NMEA_MSG_HDR_HDT[] = "HDT";

uint8_t nmea_input_buffer[NMEA_INPUT_BUFFER_SIZE];
int nmea_input_buffer_i = 0;
int nmea_received_messages = 0;





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
	//PORTB = end;
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
