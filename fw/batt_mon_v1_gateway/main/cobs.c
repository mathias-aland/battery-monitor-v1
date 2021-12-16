/*
 * cobs.c
 *
 *  Created on: 17 sep. 2021
 *      Author: Mathias
 */

#include "esp_rom_crc.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "common.h"

// decode COBS data and check CRC
bool cobs_decode(uint8_t bufLen, uint8_t *buf)
{
	uint8_t cobs_offset = 0;
	uint8_t cobs_dataCnt = 0;
	uint8_t currByte;

	if ((bufLen < (PKT_MINLEN+1)) || (bufLen > (PKT_MAXLEN+1)))
		return false;	// under PKT_MINLEN or over PKT_MAXLEN is not allowed

	// second byte is the packet length, zero is not allowed so no need to send through decoder
	// calculate the final packet length

	// check packet length
	if (buf[1] != (bufLen-1))
	{
		// packet length does not match actual received number of bytes
		return false;
	}

	while (cobs_dataCnt < bufLen)
	{

		//currByte = *buf;
		currByte = buf[cobs_dataCnt];

		// COBS decode

		// check if current byte is a COBS pointer
		if (cobs_dataCnt == cobs_offset)
		{
			// zero byte, calculate offset to next zero byte

			if (currByte > (bufLen-cobs_offset))
			{
				// invalid offset
				return false;
			}

			cobs_offset += currByte;
			buf[cobs_dataCnt] = 0;	// replace with zero in buffer
			currByte = 0;
		}

		cobs_dataCnt++;

	}

	uint16_t crc16calc = ~esp_rom_crc16_be((uint16_t)~0xffff, buf+1, bufLen-3);

	uint16_t crc16 = (buf[bufLen-2] << 8) | (buf[bufLen-1]);

	if (crc16 != crc16calc)
	{
		return false;
	}
	else
		return true;
}

// prepare buffer with COBS and CRC
bool cobs_encode(uint8_t bufLen, uint8_t *buf)
{
	uint8_t cobsIndex = 0;
	uint8_t readIndex = 1;
	uint8_t cobsCnt = 1;
	uint8_t currByte;

	//if ((bufLen < (PKT_MINLEN+1)) || (bufLen > (PKT_MAXLEN+1)))
	//	return false;	// under PKT_MINLEN or over PKT_MAXLEN is not allowed

	uint16_t crc16calc = ~esp_rom_crc16_be((uint16_t)~0xffff, buf+1, bufLen-3);

	buf[bufLen-2] = crc16calc >> 8;		// CRC MSB
	buf[bufLen-1] = crc16calc & 0xff;	// CRC LSB

	buf[0] = 0;	// make sure the first byte is 0

	while(readIndex < bufLen)
	{

		currByte = buf[readIndex];

		if (currByte == 0)
		{
			buf[cobsIndex] = cobsCnt;
			cobsCnt = 1;
			cobsIndex = readIndex;
		}
		else
		{
			cobsCnt++;
		}

		readIndex++;

	}

	// final COBS header
	buf[cobsIndex] = cobsCnt;

	return true;

}



