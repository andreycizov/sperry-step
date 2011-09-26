#include <util/atomic.h>

#ifndef F_CPU
#define F_CPU 4000000
#endif
#define USART_INPUT_BUFFER_SIZE 128
#define USART_INPUT_BUFFER_MASK (USART_INPUT_BUFFER_SIZE-1)
uint8_t usart_input_buffer[USART_INPUT_BUFFER_SIZE];
int usart_input_buffer_w = 0;
int usart_input_buffer_r = 0;

void usart_init(uint32_t baudrate)
{
	uint32_t ubrr = (F_CPU/(baudrate * 16UL) - 1);
	//Set baud rate
	UBRRL=ubrr;		//low byte
	UBRRH=(ubrr>>8);	//high byte
	//Set data frame format: asynchronous mode,no parity, 1 stop bit, 8 bit size
	UCSRC=(1<<URSEL)|(0<<UMSEL)|(0<<UPM1)|(0<<UPM0)|
		(0<<USBS)|(0<<UCSZ2)|(1<<UCSZ1)|(1<<UCSZ0);	
	//Enable Transmitter and Receiver and Interrupt on receive complete
	UCSRB=(1<<RXEN)|(0<<TXEN)|(1<<RXCIE);
	//enable global interrupts
	
}

ISR(USART_RXC_vect)
{
	uint8_t t = UDR;
	
	int next_usart_input_buffer_w = (usart_input_buffer_w + 1) & USART_INPUT_BUFFER_MASK;
	if(next_usart_input_buffer_w != usart_input_buffer_r) {
		usart_input_buffer_w = next_usart_input_buffer_w;
		usart_input_buffer[usart_input_buffer_w] = t;
	}
	//UDR=t;
}

// reads max count bytes to to
int usart_read(uint8_t *to, int count) {
	int n = 0;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
   {
		int next_usart_input_buffer_r = (usart_input_buffer_r + 1) & USART_INPUT_BUFFER_MASK;
		while(usart_input_buffer_r != usart_input_buffer_w && n < count) {
			usart_input_buffer_r = next_usart_input_buffer_r;
			to[n] = usart_input_buffer[usart_input_buffer_r]; //TODO: edit that
			next_usart_input_buffer_r = (usart_input_buffer_r + 1) & USART_INPUT_BUFFER_MASK;
			n++;
		}
	}
	return n;
}
// blocking read
void usart_read_block(uint8_t *to, int count) {
	while(count > 0) {
		int n = usart_read(to, count);
		to += n;
		count -= n;
	}
}


