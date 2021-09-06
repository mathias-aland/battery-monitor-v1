/*
 * cobs.c
 *
 *  Created on: 4 sep. 2021
 *      Author: Mathias
 */
#include <SI_EFM8BB1_Register_Enums.h>                // SFR declarations
#include <stdint.h>
#include "common.h"

// decode COBS data and check CRC
bool cobs_decode(uint8_t bufLen, uint8_t xdata *buf)
{
	uint8_t cobs_offset = 0;
	uint8_t cobs_dataCnt = 0;
	uint8_t currByte;
	uint8_t crc;

	if ((bufLen < (PKT_MINLEN)) || (bufLen > (PKT_MAXLEN)))
		return false;	// under PKT_MINLEN or over PKT_MAXLEN is not allowed

	// second byte is the packet length, zero is not allowed so no need to send through decoder
	// calculate the final packet length

	// check packet length
	if (buf[1] != (bufLen-1))
	{
		// packet length does not match actual received number of bytes
		return false;
	}

	// init CRC

	CRC0CN0 |= CRC0CN0_CRCVAL__SET_ONES;
	CRC0CN0 |= CRC0CN0_CRCINIT__INIT;
	CRC0CN0 &= ~CRC0CN0_CRCPNT__BMASK;	// lower byte first

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


		if (cobs_dataCnt >= (bufLen-2))
		{
			// CRC check
			crc = CRC0DAT;
			if (crc != currByte)
			{
				// CRC error
				//return false;
			}
		}
		else if (cobs_dataCnt != 0)	// no CRC on first byte in buffer
			CRC0IN = currByte;

		cobs_dataCnt++;

	}

	return true;
}

// prepare buffer with COBS and CRC
bool cobs_encode(uint8_t bufLen, uint8_t xdata *buf)
{
	uint8_t cobsIndex = 0;
	uint8_t readIndex = 1;
	uint8_t cobsCnt = 1;
	uint8_t currByte;

	if ((bufLen < (PKT_MINLEN)) || (bufLen > (PKT_MAXLEN)))
		return false;	// under PKT_MINLEN or over PKT_MAXLEN is not allowed

	// init CRC
	CRC0CN0 |= CRC0CN0_CRCVAL__SET_ONES;
	CRC0CN0 |= CRC0CN0_CRCINIT__INIT;

	buf[0] = 0;	// make sure the first byte is 0

	while(readIndex < bufLen)
	{

		currByte = buf[readIndex];

		if (readIndex < (bufLen-2))
		{
			// feed data to CRC module
			CRC0IN = currByte;
		}

		if (readIndex == (bufLen-3))
		{
			// CRC done, write values to buffer
			CRC0CN0 &= ~CRC0CN0_CRCPNT__BMASK;	// lower byte first
			buf[bufLen-2] = CRC0DAT;	// lower byte
			buf[bufLen-1] = CRC0DAT;	// upper byte
		}

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
