#include <util/atomic.h>
#include "circular.h"

#define USART_RX_BUFF_SIZE 128
#define USART_TX_BUFF_SIZE 128

unsigned char usart_rx_buff[USART_RX_BUFF_SIZE];
unsigned char usart_tx_buff[USART_TX_BUFF_SIZE];

circular usart_rx;
circular usart_tx;

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
	
	circular_init(&usart_rx, &usart_rx_buff, USART_RX_BUFF_SIZE);
	circular_init(&usart_tx, &usart_tx_buff, USART_TX_BUFF_SIZE);
}

ISR(USART_RXC_vect)
{
	uint8_t t = UDR;
	
	circular_write(&usart_rx, &t, 1);
}

// reads max count bytes to to
int usart_read(uint8_t *buffer, int size) {
	ATOMIC_BLOCK(ATOMIC_FORCEON)
  {
		return circular_read(&usart_rx, buffer, size);
	}
}
// blocking read
void usart_read_block(uint8_t *to, int count) {
	while(count > 0) {
		int n = usart_read(to, count);
		to += n;
		count -= n;
	}
}


ISR(USART_TXC_vect)
{
}
