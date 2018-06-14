/*
 * ds18b20Uart.c
 *
 * Created: 31.05.2018 13:15:16
 * Author : temp
 */ 
#include "main.h"
#include "onewire.h"
#include "ds18x20.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <string.h>
#include <stdint.h>

#define uart_puts_P(__s)       uart_puts_p(PSTR(__s))
/*прототипы функций*/
static void uart_init(uint16_t boudrate);
static void uart_putc(char c);
static uint8_t search_sensors(void);
static void uart_put_temp(int16_t decicelsius);

uint8_t gSensorID[OW_ROMCODE_SIZE];

static void uart_init(uint16_t boudrate) {
	uint16_t UBRRL_value = (F_CPU/(boudrate*16L))-1; //Согластно заданной скорости подсчитываем значение для регистра UBRR
	UBRRL = UBRRL_value;       //Младшие 8 бит UBR
	UBRRH = UBRRL_value >> 8;  //Старшие 8 бит UBR
	UCSRB |=(1<<TXEN);      //Бит разрешения передачи
	UCSRC |=(1<< URSEL)|(1<< UCSZ0)|(1<< UCSZ1); //Устанавливем формат 8 бит данных
}

static void uart_putc(char c) {
	while(!( UCSRA & (1 << UDRE)));   // Ожидаем когда очистится буфер передачи
	UDR = c; // Помещаем данные в буфер, начинаем передачу
}


void uart_puts_p(const char *progmem_s )
{
	register char c;
	
	while ( ( c = pgm_read_byte(progmem_s++) ) ) {
		uart_putc(c);
	}

}


static uint8_t search_sensors(void)
{
	uint8_t id[OW_ROMCODE_SIZE];
	uint8_t i,diff,nSensors;
	uart_puts_P("Start searching device\r\n");
	
	ow_reset();
	
	//search sensor
	nSensors = 0;
	diff = OW_SEARCH_FIRST;
	while((diff != OW_LAST_DEVICE)&&(nSensors==0)){
		DS18X20_find_sensor( &diff, &id[0] );
		if( diff == OW_PRESENCE_ERR ) {
			uart_puts_P( "No Sensor found \r\n" );
			break;
		}
	
		if( diff == OW_DATA_ERR ) {
			uart_puts_P( "Bus Error\r\n");
			break;
		}
	
		for ( i=0; i < OW_ROMCODE_SIZE; i++ ){
			gSensorID[i] = id[i];
		}
		nSensors++;
	}
	uart_puts_P( "found \r\n" );
	return nSensors;
}


static void uart_puts(const char *s )
{
	while ( *s ) {
		uart_putc(*s++);
	}

}


#if 0
static void uart_put_temp(int16_t decicelsius)
{
	char s[10];
	
	DS18X20_format_from_decicelsius( decicelsius, s, 10 );
	uart_puts( s );
	uart_puts_P("\r\n");
}
#else
/*Protocol constants*/
#define START_BYTE	(0x55)
#define STOP_BYTE	(0xAA)

static void uart_put_temp(int16_t decicelsius)
{
	uint8_t hi = decicelsius>>8;
	uint8_t low = decicelsius&0xFF;
	uart_putc(START_BYTE);
	uart_putc(hi);
	uart_putc(low);
	uart_putc((uint8_t)(hi+low)); // control summ
	uart_putc(STOP_BYTE);
}
#endif

int main(void)
{
	int16_t decicelsius;
	/*инициализация переферии*/
	uart_init(UART_BAUDRATE);
	sei();
	search_sensors();
	if ( gSensorID[0] == DS18S20_FAMILY_CODE ) {
		uart_puts_P( "DS18S20/DS1820" );
	} else if ( gSensorID[0] == DS1822_FAMILY_CODE ) {
		uart_puts_P( "DS1822" );
	}else {
		uart_puts_P( "DS18B20" );
	}
	uart_puts_P( "\r\n" );
    while (1) {
		DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL );
		_delay_ms( DS18B20_TCONV_12BIT );
		DS18X20_read_decicelsius_single( gSensorID[0]/*family in first byte*/, &decicelsius );
		uart_put_temp( decicelsius );

    }
}

