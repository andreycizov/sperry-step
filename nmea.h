
#define NMEA_INPUT_BUFFER_SIZE 64

#define NMEA_SYMBOL_START '$'
#define NMEA_SYMBOL_CHECKSUM '*'
#define NMEA_SYMBOL_END 0x0A

uint8_t nmea_input_buffer[NMEA_INPUT_BUFFER_SIZE];
int nmea_input_buffer_i = 0;

uint8_t nmea_checksum(uint8_t *d, int length) {
	uint8_t cs = *d;
	while(length > 0) {
		length--;
		d++;
		cs ^= *d;
	}
	return cs;
}

int nmea_msg_symbol(uint8_t symbol, uint8_t *d, int start, int count) {
	for(int i = start; i < start+count; i++) {
		if(d[i] == symbol)
			return i;
	}
	return -1;
}
uint8_t nmea_checksums = 0;
int nmea_process(uint8_t *d, int count) {
	int end = nmea_msg_symbol(NMEA_SYMBOL_END, d, 0, count);
	int end_prev = -1;
	while( end >= 0 ) {
		int start = nmea_msg_symbol(NMEA_SYMBOL_START, d, end, count - end - 1) + 1;
		int checksum = nmea_msg_symbol(NMEA_SYMBOL_CHECKSUM, d, start, end - start);
		
		int cs = nmea_checksum(d + start, checksum - start);
		nmea_checksums = cs;
		end_prev = end;
		end = nmea_msg_symbol(NMEA_SYMBOL_END, d, end + 1, count - end - 2);
	}
	return end_prev;
}

void nmea_copy(uint8_t *d, int count) {
	int left = NMEA_INPUT_BUFFER_SIZE - nmea_input_buffer_i;
	count = count > left ? left:count;
	memcpy(nmea_input_buffer + nmea_input_buffer_i, d, count);
	nmea_input_buffer_i += count;
}

// reads n bytes from buffer
void nmea_read(uint8_t *d, int count) {
	if(nmea_input_buffer_i == 0) {
		int start = nmea_msg_symbol(NMEA_SYMBOL_START, d, 0, count);
		nmea_copy(d + start, count - start);
		if(start == -1) {
			return;
		}
	} else {
		nmea_copy(d, count);
	}
	
	int end = nmea_process(nmea_input_buffer, nmea_input_buffer_i + 1);
				
	if((end == -1) && ((nmea_input_buffer_i + 1) == NMEA_INPUT_BUFFER_SIZE)) {
		nmea_input_buffer_i = 0;
		return;
	} else if (end >= 0) {
		nmea_copy(nmea_input_buffer + end + 1, NMEA_INPUT_BUFFER_SIZE - end - 2);
		nmea_input_buffer_i = 0;	
	}
}
