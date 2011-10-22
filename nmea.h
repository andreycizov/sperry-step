#include "degree.h"

#define NMEA_INPUT_BUFFER_SIZE 64
#define NMEA_OUTPUT_BUFFER_SIZE 32

#define NMEA_MSG_MIN_SIZE 12
#define NMEA_MSG_SIZE(x) (x + NMEA_MSG_MIN_SIZE)


#define NMEA_MSG_HEADER_SIZE 5
#define NMEA_MSG_CHECKSUM_SIZE 2

#define NMEA_MSG_BODY_OFFSET (1 /* msg start */ + NMEA_MSG_HEADER_SIZE + 1 /* comma */)

#define NMEA_SYMBOL_START '$'
#define NMEA_SYMBOL_CHECKSUM '*'
#define NMEA_SYMBOL_END2 0x0D
#define NMEA_SYMBOL_END 0x0A

#define NMEA_SYMBOL_FIELD_SEP ','
#define NMEA_SYMBOL_POINT_SEP '.'

#define NMEA_SYMBOL_HEADING_TRUE 'T'

#define NMEA_SYMBOL_0 0x30
#define NMEA_SYMBOL_9 0x39
#define NMEA_SYMBOL_A 0x41
#define NMEA_SYMBOL_F 0x46
#define NMEA_SYMBOL_IS_DECIMAL(s) (s >= NMEA_SYMBOL_0 && s <= NMEA_SYMBOL_9)

#define NMEA_MSG_HDR_SIZE 3
const char NMEA_MSG_HDR_HDT[] = "HDT";
const char NMEA_MSG_HDR_HEHDT[] = "HEHDT";

uint8_t nmea_input_buffer[NMEA_INPUT_BUFFER_SIZE];
int nmea_input_buffer_i = 0;
int nmea_received_messages = 0;

uint8_t nmea_msg_forward_buffer[NMEA_OUTPUT_BUFFER_SIZE];

uint8_t nmea_checksum(uint8_t *d, int length);


int nmea_msg_atoi_size(uint8_t *d, int count) {
	int size = 0;
	for(int i = 0;
	  NMEA_SYMBOL_IS_DECIMAL(d[i]) &&
	  i < count; i++){
		size++;
	}
	return size;
}

uint32_t nmea_msg_atoi(uint8_t *d, int count) {
	uint32_t mul = 1;
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

	d += size_i;

	if(d[0] != NMEA_SYMBOL_POINT_SEP)
		return -1;

	d++;
	count -= 1;

	int size_frac = nmea_msg_atoi_size(d, count);

	r->frac = nmea_msg_atoi(d, size_frac);

	r->denom = 1;
	for(int i = 0; i < size_frac; i++ )
		r->denom *= 10;
	// Integer + Fraction + Point
	return size_i + size_frac + 1;
}
int nmea_field_process(uint8_t *d, int count) {
	if(count == 0 || *d != NMEA_SYMBOL_FIELD_SEP)
		return -1;
	d++;
	int i = 0;
	while(d[i] != NMEA_SYMBOL_FIELD_SEP && i < count)
		i++;
	return i;
}

int nmea_msg_hdr_cmp(uint8_t *d, const char *cmp, int count) {
	for(int i = 0; i < count; i++ ){
		if(d[i] != cmp[i])
			return 0;
	}
	return 1;
}

typedef struct heading {
	degree degr;
	int degr_start;
	int degr_size;
	int letter_start;
	int letter_size;
} heading;

int nmea_msg_process_heading(uint8_t *d, int count, heading *r) {
	if(count == 0)
		return 0;

	// process degr field

	int f_degr = nmea_field_process(d, count);
	if(f_degr == -1)
		return 0;

	int degr_size = nmea_msg_ansi_to_degr(d + 1, f_degr, &(r->degr));

	if(degr_size != f_degr)
		return 0;

	// end process degr field

	int f_letter = nmea_field_process(d + f_degr + 1, count -= f_degr + 1);

	r->degr_start = 1;
	r->degr_size = f_degr;
	r->letter_start = f_degr + 1;
	r->letter_size = f_letter;

	return 1;
}

void nmea_msg_process_hdt(uint8_t *d, int count) {
	heading h;
	if(nmea_msg_process_heading(d, count, &h)) {
		global_degr_update(h.degr);
		nmea_msg_forward_heading(d, &h);
	}
}

void nmea_msg_process_ths(uint8_t *d, int count) {
	heading h;
	if(nmea_msg_process_heading(d, count, &h)) {
		nmea_msg_forward_heading(d, &h);
	}
}

void nmea_msg_process_hdm(uint8_t *d, int count) {
	heading h;
	if(nmea_msg_process_heading(d, count, &h)) {
		nmea_msg_forward_heading(d, &h);
	}
}

// returns the offset where to write the body
// gets the message size in bytes
// @args:
// type-> 5-byte message header;
// msg-> message buffer
// size-> message buffer size
// count-> message body size
int nmea_msg_prepare_header(uint8_t *msg, uint8_t *type, int size, int count) {
	if(NMEA_MSG_SIZE(count) < size)
		return -1;
	msg[0] = NMEA_SYMBOL_START;
	memcpy(msg += 1, type, NMEA_MSG_HEADER_SIZE);
	msg += NMEA_MSG_HEADER_SIZE + size + 1;
	msg[0] = NMEA_SYMBOL_CHECKSUM;
	msg += NMEA_MSG_CHECKSUM_SIZE + 1;
	msg[0] = NMEA_SYMBOL_END2;
	msg[1] = NMEA_SYMBOL_END;
	return NMEA_MSG_BODY_OFFSET;
}

void nmea_msg_prepare_checksum(uint8_t *msg, int size, uint8_t *d, int count) {
	int offset = NMEA_MSG_BODY_OFFSET + size + 1;
	nmea_msg_byte_to_hex(nmea_checksum(d, count), msg + offset);
}

void nmea_msg_forward_heading(uint8_t *d, heading *h) {
	int r = nmea_msg_prepare_header(
		nmea_msg_forward_buffer, 
		NMEA_MSG_HDR_HEHDT,
		NMEA_OUTPUT_BUFFER_SIZE,
		h->degr_size + 1 + 1);

	if(r < 0)
		return; // too big message, cannot send it!!
	uint8_t *out = &nmea_msg_forward_buffer + r;
	memcpy(out, d + h->degr_start, h->degr_size);
	out += h->degr_size; 
	out[0] = NMEA_SYMBOL_FIELD_SEP;
    out[1] = NMEA_SYMBOL_HEADING_TRUE;

	usart_write(out, NMEA_MSG_SIZE(h->degr_size + 1 + 1));
}

int16_t nmea_msg_hex_to_byte(uint8_t *d) {
	uint8_t r = 0;
	for(int
	 i = 4; i >= 0; i-=4 ) {
		if(NMEA_SYMBOL_IS_DECIMAL(*d)) { // *d in [0..9]
			r+= ((*d - NMEA_SYMBOL_0) << i);
		} else if (*d >= NMEA_SYMBOL_A && *d <= NMEA_SYMBOL_F) { // *d in [A..F]
			r+= ((*d - NMEA_SYMBOL_A + 10) << i);
		} else {
			return -1;
		}
		d++;
	}
	return (int16_t)r;
}

void nmea_msg_byte_to_hex(uint8_t cs, uint8_t *d) {
	d[0] = '0';
	d[1] = '1';
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



// input is in brackets: $[*d]*XX<CR><ENDL>
void nmea_msg_process(uint8_t *d, int count) {
	uint8_t *hdr = d + 2;
	uint8_t *body = d + 5;
	int body_size = count - 5;

	if(nmea_msg_hdr_cmp(hdr, NMEA_MSG_HDR_HDT, NMEA_MSG_HDR_SIZE))
		nmea_msg_process_hdt(body, body_size);
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
			// Check whole datagram length here
			if( start == -1 || end - start < 10 )
				break;
			d += start + 1;
			end -= start + 1;

			int checksum = nmea_msg_symbol(NMEA_SYMBOL_CHECKSUM, d, end);

			// Difference between checksum and message end is 4 bytes.
			// Message body length also cannot be lower than 5 bytes,
			// it's length is equal to checksum index
			if( checksum == -1 || end - checksum != 4 || checksum < 5 )
				break;

			// Checksum checking
			if( ((int16_t)nmea_checksum(d, checksum)) != nmea_msg_hex_to_byte(d+checksum+1) )
				break;

			// The message is OK, receive it.
			nmea_received_messages++;

			nmea_msg_process(d, checksum);

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
