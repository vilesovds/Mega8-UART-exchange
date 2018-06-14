/*
 * slave.c
 *
 * Created: 13.06.2018 13:23:53
 * Author : temp
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
// HW defines
#define DDR_LED (DDRB)
#define PORT_LED (PORTB)
#define PIN_LED (1<<0)

#ifndef F_CPU
#define F_CPU (8000000UL)
#endif

#ifndef UART_BAUDRATE
#define UART_BAUDRATE (9600)
#endif

/*Protocol constants*/
#define START_BYTE	(0x55)
#define STOP_BYTE	(0xAA)




/*prototypes*/
static void uart_init(uint16_t boudrate);
static void parce_process(uint8_t new_data);
static void gpio_init(void);

static volatile uint8_t  new_data =0; // new data package flag
static const int16_t ustavka = -140;// дециЦельсии, т.е. температура, умноженная на 10
static volatile int16_t value  = 0;
static int16_t payload = 0;

static void uart_init(uint16_t boudrate) {
	uint16_t UBRRL_value = (F_CPU/(boudrate*16L))-1; //Согластно заданной скорости подсчитываем значение для регистра UBRR
	UBRRL = UBRRL_value;       //Младшие 8 бит UBR
	UBRRH = UBRRL_value >> 8;  //Старшие 8 бит UBR
    UCSRB = ((1<<RXEN) | (1<<RXCIE)); // Enable receiver  and receive complete interrupt
	UCSRC |=(1<< URSEL)|(1<< UCSZ0)|(1<< UCSZ1); //Устанавливем формат 8 бит данных
}

/**
*Good package: Start byte + payload + control summ + Stop byte
*/
void parce_process(uint8_t data)
{
	static uint8_t parce_state = 0;

	switch(parce_state){
		case 1:
			payload=(int16_t)data<<8; // high byte
			parce_state = 2; // next state
			break;
	    case 2:
			payload|=data; // low byte
			parce_state = 3; // next state
			break;
		case 3:
			//check control summ
			{
				uint8_t byte1 = payload>>8; 
				uint8_t byte2 = payload&(0xFF);
				if( (uint8_t)(byte1+byte2)==data){
					parce_state = 4; // next state
				}else{
					//error
					parce_state = 0;
				}
			}
			break;
		case 4:
			if(STOP_BYTE==data){
				value = payload; // copy data
				new_data = 1;// set flag
			}
		    parce_state = 0;
			break;
		default:
			payload = 0;
			if(START_BYTE==data) {
				parce_state = 1;
			}
		break;
	}
	
}

ISR(USART_RXC_vect)
{
	uint8_t d = UDR;
	parce_process(d);
}

static void gpio_init(void)
{
	DDR_LED|=PIN_LED;
	
}

int main(void)
{
    /*инициализация переферии*/
	gpio_init();
    uart_init(UART_BAUDRATE);
    sei();
    while (1){
		if(new_data){
			if(value>ustavka){
				// turn on LED
				PORT_LED|=PIN_LED;
			}else{
				//turn off LED
				PORT_LED&=~PIN_LED;
			}
			new_data = 0;// clear flag
		}
    }
}

