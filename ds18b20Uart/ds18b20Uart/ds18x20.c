#include "ds18x20.h"
#include "onewire.h"
#include "crc8.h"

static int16_t DS18X20_raw_to_decicelsius( uint8_t familycode, uint8_t sp[] );
static uint8_t read_scratchpad( uint8_t id[], uint8_t sp[], uint8_t n );

/* find DS18X20 Sensors on 1-Wire-Bus
   input/ouput: diff is the result of the last rom-search
                *diff = OW_SEARCH_FIRST for first call
   output: id is the rom-code of the sensor found */
uint8_t DS18X20_find_sensor( uint8_t *diff, uint8_t id[] )
{
	uint8_t go;
	uint8_t ret;

	ret = DS18X20_OK;
	go = 1;
	do {
		*diff = ow_rom_search( *diff, &id[0] );
		if ( *diff == OW_PRESENCE_ERR || *diff == OW_DATA_ERR ||
		     *diff == OW_LAST_DEVICE ) { 
			go  = 0;
			ret = DS18X20_ERROR;
		} else {
			if ( id[0] == DS18B20_FAMILY_CODE || id[0] == DS18S20_FAMILY_CODE ||
			     id[0] == DS1822_FAMILY_CODE ) { 
				go = 0;
			}
		}
	} while (go);

	return ret;
}

/* start measurement (CONVERT_T) for all sensors if input id==NULL 
   or for single sensor where id is the rom-code */
uint8_t DS18X20_start_meas( uint8_t with_power_extern, uint8_t id[])
{
	uint8_t ret;

	ow_reset();
	if( ow_input_pin_state() ) { // only send if bus is "idle" = high
		if ( with_power_extern != DS18X20_POWER_EXTERN ) {
			ow_command_with_parasite_enable( DS18X20_CONVERT_T, id );
			/* not longer needed: ow_parasite_enable(); */
		} else {
			ow_command( DS18X20_CONVERT_T, id );
		}
		ret = DS18X20_OK;
	} 
	else { 
		//uart_puts_P_verbose( "DS18X20_start_meas: Short Circuit!\r" );
		ret = DS18X20_START_FAIL;
	}

	return ret;
}


/* convert scratchpad data to physical value in unit decicelsius */
static int16_t DS18X20_raw_to_decicelsius( uint8_t familycode, uint8_t sp[] )
{
	uint16_t measure;
	uint8_t  negative;
	int16_t  decicelsius;
	uint16_t fract;

	measure = sp[0] | (sp[1] << 8);
	//measure = 0xFF5E; // test -10.125
	//measure = 0xFE6F; // test -25.0625

	if( familycode == DS18S20_FAMILY_CODE ) {   // 9 -> 12 bit if 18S20
		/* Extended measurements for DS18S20 contributed by Carsten Foss */
		measure &= (uint16_t)0xfffe;   // Discard LSB, needed for later extended precicion calc
		measure <<= 3;                 // Convert to 12-bit, now degrees are in 1/16 degrees units
		measure += (16 - sp[6]) - 4;   // Add the compensation and remember to subtract 0.25 degree (4/16)
	}

	// check for negative
	if ( measure & 0x8000 )  {
		negative = 1;       // mark negative
		measure ^= 0xffff;  // convert to positive => (twos complement)++
		measure++;
	}
	else {
		negative = 0;
	}

	// clear undefined bits for DS18B20 != 12bit resolution
	if ( familycode == DS18B20_FAMILY_CODE || familycode == DS1822_FAMILY_CODE ) {
		switch( sp[DS18B20_CONF_REG] & DS18B20_RES_MASK ) {
			case DS18B20_9_BIT:
			measure &= ~(DS18B20_9_BIT_UNDF);
			break;
			case DS18B20_10_BIT:
			measure &= ~(DS18B20_10_BIT_UNDF);
			break;
			case DS18B20_11_BIT:
			measure &= ~(DS18B20_11_BIT_UNDF);
			break;
			default:
			// 12 bit - all bits valid
			break;
		}
	}

	decicelsius = (measure >> 4);
	decicelsius *= 10;

	// decicelsius += ((measure & 0x000F) * 640 + 512) / 1024;
	// 625/1000 = 640/1024
	fract = ( measure & 0x000F ) * 640;
	if ( !negative ) {
		fract += 512;
	}
	fract /= 1024;
	decicelsius += fract;

	if ( negative ) {
		decicelsius = -decicelsius;
	}

	if ( /* decicelsius == 850 || */ decicelsius < -550 || decicelsius > 1250 ) {
		return DS18X20_INVALID_DECICELSIUS;
		} else {
		return decicelsius;
	}
}

/* reads temperature (scratchpad) of sensor with rom-code id
   output: decicelsius 
   returns DS18X20_OK on success */
uint8_t DS18X20_read_decicelsius( uint8_t id[], int16_t *decicelsius )
{
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;
	
	ow_reset();
	ret = read_scratchpad( id, sp, DS18X20_SP_SIZE );
	if ( ret == DS18X20_OK ) {
		*decicelsius = DS18X20_raw_to_decicelsius( id[0], sp );
	}
	return ret;
}


static uint8_t read_scratchpad( uint8_t id[], uint8_t sp[], uint8_t n )
{
	uint8_t i;
	uint8_t ret;

	ow_command( DS18X20_READ, id );
	for ( i = 0; i < n; i++ ) {
		sp[i] = ow_byte_rd();
	}
	if ( crc8( &sp[0], DS18X20_SP_SIZE ) ) {
		ret = DS18X20_ERROR_CRC;
		} else {
		ret = DS18X20_OK;
	}

	return ret;
}

/* reads temperature (scratchpad) of sensor without id (single sensor)
   output: decicelsius 
   returns DS18X20_OK on success */
uint8_t DS18X20_read_decicelsius_single( uint8_t familycode, int16_t *decicelsius )
{
	uint8_t sp[DS18X20_SP_SIZE];
	uint8_t ret;
	
	ret = read_scratchpad( NULL, sp, DS18X20_SP_SIZE );
	if ( ret == DS18X20_OK ) {
		*decicelsius = DS18X20_raw_to_decicelsius( familycode, sp );
	}
	return ret;
}


/* format decicelsius-value into string, itoa method inspired 
   by code from Chris Takahashi for the MSP430 libc, BSD-license 
   modifications mthomas: variable-types, fixed radix 10, use div(), 
   insert decimal-point */
uint8_t DS18X20_format_from_decicelsius( int16_t decicelsius, char str[], uint8_t n)
{
	uint8_t sign = 0;
	char temp[7];
	int8_t temp_loc = 0;
	uint8_t str_loc = 0;
	div_t dt;
	uint8_t ret;

	// range from -550:-55.0�C to 1250:+125.0�C -> min. 6+1 chars
	if ( n >= (6+1) && decicelsius > -1000 && decicelsius < 10000 ) {

		if ( decicelsius < 0) {
			sign = 1;
			decicelsius = -decicelsius;
		}

		// construct a backward string of the number.
		do {
			dt = div(decicelsius,10);
			temp[temp_loc++] = dt.rem + '0';
			decicelsius = dt.quot;
		} while ( decicelsius > 0 );

		if ( sign ) {
			temp[temp_loc] = '-';
		} else {
			///temp_loc--;
			temp[temp_loc] = '+';
		}

		// reverse the string.into the output
		while ( temp_loc >=0 ) {
			str[str_loc++] = temp[(uint8_t)temp_loc--];
			if ( temp_loc == 0 ) {
				str[str_loc++] = DS18X20_DECIMAL_CHAR;
			}
		}
		str[str_loc] = '\0';

		ret = DS18X20_OK;
	} else {
		ret = DS18X20_ERROR;
	}
	
	return ret;
}
