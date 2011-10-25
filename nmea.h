#include "degree.h"

#define NMEA_INPUT_BUFFER_SIZE 64
#define NMEA_OUTPUT_BUFFER_SIZE 48

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
const char NMEA_MSG_HDR_THS[] = "THS";
const char NMEA_MSG_HDR_HDM[] = "HDM";

const char NMEA_MSG_HDR_HEHDT[] = "HEHDT";

uint8_t nmea_input_buffer[NMEA_INPUT_BUFFER_SIZE];
int nmea_input_buffer_i = 0;
int nmea_received_messages = 0;

int nmea_require_checksum = 1;

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

int nmea_msg_process_heading_degree(uint8_t *d, int count, heading *r) {
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

void nmea_msg_process_heading(uint8_t *d, int count) {
	heading h;
	if(nmea_msg_process_heading_degree(d, count, &h)) {
		global_degr_update(h.degr);
		global_degr_str_update(d + h.degr_start, h.degr_size);
		//nmea_msg_forward_heading(d, &h);
	}
}

void nmea_msg_process_hdt(uint8_t *d, int count) {
	nmea_msg_process_heading(d, count);
}

void nmea_msg_process_ths(uint8_t *d, int count) {
	nmea_msg_process_heading(d, count);
}

void nmea_msg_process_hdm(uint8_t *d, int count) {
	nmea_msg_process_heading(d, count);
}

typedef struct nmea_msg {
	uint8_t checksum;
} nmea_msg;

void nmea_msg_tx_body(nmea_msg *h, uint8_t *body, int count) {
	h->checksum ^= nmea_checksum(body, count);
	usart_write(body, count);
}

void nmea_msg_tx_start(nmea_msg *h, uint8_t *header) {
	h->checksum = 0;	
	uint8_t start[7];
	start[0] = NMEA_SYMBOL_START;
	usart_write(start, 1);
	memcpy(start + 1, header, NMEA_MSG_HEADER_SIZE);
	start[6] = NMEA_SYMBOL_FIELD_SEP;
	nmea_msg_tx_body(h, start + 1, NMEA_MSG_HEADER_SIZE + 1);
}

void nmea_msg_tx_end(nmea_msg *h) {
	uint8_t end[5];
	end[0] = NMEA_SYMBOL_CHECKSUM;
	nmea_msg_byte_to_hex(h->checksum, end + 1);
	end[3] = NMEA_SYMBOL_END2;
	end[4] = NMEA_SYMBOL_END;
	usart_write(end, sizeof(end));
}

void nmea_msg_forward_heading(uint8_t *heading, int length) {
	nmea_msg msg;
	nmea_msg_tx_start(&msg, NMEA_MSG_HDR_HEHDT);
	nmea_msg_tx_body(&msg, heading, length);

	uint8_t f_hdg_true[2];
	f_hdg_true[0] = NMEA_SYMBOL_FIELD_SEP;
	f_hdg_true[1] = 'T';

	nmea_msg_tx_body(&msg, f_hdg_true, sizeof(f_hdg_true));
	nmea_msg_tx_end(&msg);
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
	for(int i = 4; i >= 0; i -= 4) {
		unsigned int val = (cs >> i) & 0x0F;
		if(val < 10) {
			d[0] = NMEA_SYMBOL_0 + val;
		} else {
			d[0] = NMEA_SYMBOL_A + val - 10;
		}
		d++;
	}
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

	if(nmea_msg_hdr_cmp(hdr, NMEA_MSG_HDR_HDT, NMEA_MSG_HDR_SIZE)) {
		nmea_msg_process_hdt(body, body_size);
	}
	else if(nmea_msg_hdr_cmp(hdr, NMEA_MSG_HDR_THS, NMEA_MSG_HDR_SIZE)) {
		nmea_msg_process_ths(body, body_size);
	}
	else if(nmea_msg_hdr_cmp(hdr, NMEA_MSG_HDR_HDM, NMEA_MSG_HDR_SIZE)) {
		nmea_msg_process_hdm(body, body_size);
	}
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
			int msg_max = 10;

			if(!nmea_require_checksum) 
				msg_max -= 3;

			if( start == -1 || end - start < msg_max )
				break;
			d += start + 1;
			end -= start + 1;
			
			int checksum = end;
			if( nmea_require_checksum ) {
				checksum = nmea_msg_symbol(NMEA_SYMBOL_CHECKSUM, d, end);

				// Difference between checksum and message end is 4 bytes.
				// Message body length also cannot be lower than 5 bytes,
				// it's length is equal to checksum index
				if( checksum == -1 || end - checksum != 4 || checksum < 5 )
					break;

				// Checksum checking
			
				if( ((int16_t)nmea_checksum(d, checksum)) != nmea_msg_hex_to_byte(d+checksum+1) )
					break;
			}

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
