/*
 * main.c
 *
 *  Created on: 5 sep. 2021
 *      Author: Mathias
 */

#include <SI_EFM8BB1_Register_Enums.h>                // SFR declarations

#include <string.h>
#include <stdio.h>
#include <stdint.h>

#include "common.h"
#include "cobs.h"

#define PKT_FLAG_REPLY		0x80
#define PKT_FLAG_OK 		0x40

#define PKT_CMD_ID			0x00
#define PKT_CMD_STATUS		0x01
#define PKT_CMD_MEAS		0x02

#define PKT_CMD_SETADDR		0x10
#define PKT_CMD_GETCAL		0x11
#define PKT_CMD_SETCAL		0x12
#define PKT_CMD_WRITECAL	0x13


#define ADC_NUM_SAMPLES		16


#define MUX_CH1	1
#define MUX_CH2	3
#define MUX_CH3	2
#define MUX_CH4	4
#define MUX_CH5	6
#define MUX_CH6	5

#define PARAMS_LEN	33
#define PARAMS_VER	1

#define PARAM_CHECK_CRC_ERR	0x01
#define PARAM_CHECK_VER_ERR	0x02
#define PARAM_CHECK_OLD		0x04
#define PARAM_CHECK_CURRENT	0x08
#define PARAM_CHECK_ERROR	0x80

#define CAL_DATA_ADDR			0x00
#define CAL_DATA_OFFSET			0x01
#define CAL_DATA_FULLSCALE		0x11

#define CAL_PAGE_BYTES	(PARAMS_LEN+4)

#define CAL_PAGE_OFFSET_VER		0x00
#define CAL_PAGE_OFFSET_VER_INV	0x01
#define CAL_PAGE_OFFSET_CRCH	0x02
#define CAL_PAGE_OFFSET_CRCL	0x03
#define CAL_PAGE_OFFSET_DATA	0x04


#define FLASH_ADDR_CAL		0x1C00
#define FLASH_ADDR_BOOT		0x1E00

SI_LOCATED_VARIABLE_NO_INIT(uuid[4], uint8_t, const SI_SEG_XDATA, 0x00FC);

// special flash pages
SI_LOCATED_VARIABLE_NO_INIT(nvs_flash[512], uint8_t, const SI_SEG_CODE, FLASH_ADDR_CAL);
SI_LOCATED_VARIABLE_NO_INIT(boot_flash[512], uint8_t, const SI_SEG_CODE, FLASH_ADDR_BOOT);


/* --- calibration page structure ---
 * Version (1 byte)
 * Version inverted (1 byte)
 * Data CRC-16 (2 bytes)
 * CAL DATA (0-252 bytes)
 */

/* --- CAL DATA structure v1 ---
 * 0x000: Card address
 * 0x001: CH1 OFFSET MSB
 * 0x002: CH1 OFFSET LSB
 * 0x003: CH2 OFFSET MSB
 * 0x004: CH2 OFFSET LSB
 * 0x005: CH3 OFFSET MSB
 * 0x006: CH3 OFFSET LSB
 * 0x007: CH4 OFFSET MSB
 * 0x008: CH4 OFFSET LSB
 * 0x009: CH5 OFFSET MSB
 * 0x00A: CH5 OFFSET LSB
 * 0x00B: CH6 OFFSET MSB
 * 0x00C: CH6 OFFSET LSB
 * 0x00D: VDD OFFSET MSB
 * 0x00E: VDD OFFSET LSB
 * 0x00F: TEMP OFFSET MSB
 * 0x010: TEMP OFFSET LSB
 * 0x011: CH1 FULLSCALE MSB
 * 0x012: CH1 FULLSCALE LSB
 * 0x013: CH2 FULLSCALE MSB
 * 0x014: CH2 FULLSCALE LSB
 * 0x015: CH3 FULLSCALE MSB
 * 0x016: CH3 FULLSCALE LSB
 * 0x017: CH4 FULLSCALE MSB
 * 0x018: CH4 FULLSCALE LSB
 * 0x019: CH5 FULLSCALE MSB
 * 0x01A: CH5 FULLSCALE LSB
 * 0x01B: CH6 FULLSCALE MSB
 * 0x01C: CH6 FULLSCALE LSB
 * 0x01D: VDD FULLSCALE MSB
 * 0x01E: VDD FULLSCALE LSB
 * 0x01F: TEMP FULLSCALE MSB
 * 0x020: TEMP FULLSCALE LSB
 */

/* --- CAL DATA structure v2 ---
 * 0x000: CH1 OFFSET MSB
 * 0x001: CH1 OFFSET LSB
 * 0x002: CH2 OFFSET MSB
 * 0x003: CH2 OFFSET LSB
 * 0x004: CH3 OFFSET MSB
 * 0x005: CH3 OFFSET LSB
 * 0x006: CH4 OFFSET MSB
 * 0x007: CH4 OFFSET LSB
 * 0x008: CH5 OFFSET MSB
 * 0x009: CH5 OFFSET LSB
 * 0x00A: CH6 OFFSET MSB
 * 0x00B: CH6 OFFSET LSB
 * 0x00C: VDD OFFSET MSB
 * 0x00D: VDD OFFSET LSB
 * 0x00E: TEMP OFFSET MSB
 * 0x00F: TEMP OFFSET LSB
 * 0x010: CH1 FULLSCALE MSB
 * 0x011: CH1 FULLSCALE LSB
 * 0x012: CH2 FULLSCALE MSB
 * 0x013: CH2 FULLSCALE LSB
 * 0x014: CH3 FULLSCALE MSB
 * 0x015: CH3 FULLSCALE LSB
 * 0x016: CH4 FULLSCALE MSB
 * 0x017: CH4 FULLSCALE LSB
 * 0x018: CH5 FULLSCALE MSB
 * 0x019: CH5 FULLSCALE LSB
 * 0x01A: CH6 FULLSCALE MSB
 * 0x01B: CH6 FULLSCALE LSB
 * 0x01C: VDD FULLSCALE MSB
 * 0x01D: VDD FULLSCALE LSB
 * 0x01E: TEMP FULLSCALE MSB
 * 0x01F: TEMP FULLSCALE LSB
 * 0x020: Card address
 * 0x021: Bank position
 * 0x030-0x04F: Card name
 * 0x050-0x06F: Bank name
 * 0x070-0x08F: CH1 name
 * 0x090-0x0AF: CH2 name
 * 0x0B0-0x0CF: CH3 name
 * 0x0D0-0x0EF: CH4 name
 * 0x0F0-0x10F: CH5 name
 * 0x110-0x12F: CH6 name
 *
 */


uint8_t pktBuf_len = 0;
uint8_t xdata pktBuf[PKT_MAXLEN];
uint8_t ownAddr = 1;

uint8_t flash_key1 = 0;
uint8_t flash_key2 = 0;

// ch sel	input
// X7 000	GND
// X6 001	B1
// X5 010	B3
// X4 011	B2
// X3 100	B4
// X2 101	B+
// X1 110	B5
// X0 111	N/A

uint8_t code chMux[6] = {1,3,2,4,6,5};

int16_t xdata cal_offset[8] = {10,10,10,10,10,10,0,-2656};
uint16_t xdata cal_fullscale[8] = {20290,20290,20290,20290,20290,20290,4800,8421};

//-----------------------------------------------------------------------------
// SiLabs_Startup() Routine
// ----------------------------------------------------------------------------
// This function is called immediately after reset, before the initialization
// code is run in SILABS_STARTUP.A51 (which runs before main() ). This is a
// useful place to disable the watchdog timer, which is enable by default
// and may trigger before main() in some instances.
//-----------------------------------------------------------------------------
void SiLabs_Startup (void)
{
	//Disable Watchdog with key sequence
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
}

SI_INTERRUPT(INT0_ISR, INT0_IRQn)
{
	// INT0 IRQ needs to be enabled to allow wake up
	// flag cleared automatically
}

SI_INTERRUPT(ADC0_ISR, ADC0EOC_IRQn)
{
	// ADC0 EOC IRQ needs to be enabled to allow wake up
	ADC0CN0_ADINT = 0;	// Clear IRQ
}

void resetTimeout()
{
	TCON_TR0 = 0;
	TCON_TF0 = 0;

	// 100 msec at HFOSC/16 (/4)
	TH0 = 0x6A;
	TL0 = 0x76;

	// 100 msec at HFOSC/16 (/48)
	//TH0 = 0xF3;
	//TL0 = 0x8A;

	// 1 sec
	//TH0 = 0x83;
	//TL0 = 0x63;

	//TH0 = 0;
	//TL0 = 0;


}

bool sendReply()
{

	uint8_t txLen;
	uint8_t txIdx;

	txLen = pktBuf[PKT_INDEX_LEN] + 1;

	if (!cobs_encode(txLen, pktBuf))
	{
		// COBS encoding failed
		return false;
	}

	// enforce minimum RX -> TX time by waiting until Timer0 overflows
	while (!TCON_TF0);
	resetTimeout();

	SCON0_TI = 0;	// reset TI flag

	// send frame start
	SBUF0 = 0;
	while (!SCON0_TI);	// wait for TX to finish
	SCON0_TI = 0;	// reset TI flag

	// send buffer
	for (txIdx = 0; txIdx < txLen; txIdx++)
	{
		SBUF0 = pktBuf[txIdx];
		while (!SCON0_TI);	// wait for TX to finish
		SCON0_TI = 0;	// reset TI flag
	}

	// transmit completed, send frame end and reset index
	SBUF0 = 0;
	while (!SCON0_TI);	// wait for TX to finish
	SCON0_TI = 0;	// reset TI flag

	return true;
}


uint16_t getADC()
{
	uint8_t adcCnt;
	uint32_t acc = 0;

	//TMR2 =   50224; // delay first conversion start by 10 ms (HFOSC/16)
	//TMR2 =   57880; // delay first conversion start by 5 ms (HFOSC/16)
	//TMR2 =   64005; // delay first conversion start by 1 ms (HFOSC/16)
	TMR2 =   62474; // delay first conversion start by 2 ms (HFOSC/16)

	for (adcCnt = 0; adcCnt < ADC_NUM_SAMPLES; adcCnt++)
	{
		TMR2CN0_TR2 = 1;	// acquire sample

		// enter idle mode to reduce noise
		PCON0 |= 0x01;
		PCON0 = PCON0;

		TMR2CN0_TR2 = 0;
		//TMR2 =    65505; // delay conversion start by ~20 us (HFOSC/16)
		TMR2 =    65383; // delay conversion start by ~100 us (HFOSC/16)

		// process result
		acc += ADC0;

		//ADC0CN0_ADINT = 0;
	}

	return (acc / ADC_NUM_SAMPLES);	// round to 16 bits
}

void doMeas()
{
	int16_t adcRes;
	uint8_t ch;

	// measure and put into packet buffer

	ADC0MX = ADC0MX_ADC0MX__ADC0P7;	// select ADC0P7 (battery measurement)

	for (ch = 0; ch < 6; ch++)
	{
		P1 = chMux[ch];	// select MUX channel
		adcRes = (((uint32_t)getADC() * cal_fullscale[ch]) / 65536) + cal_offset[ch];	// apply calibration
		pktBuf[PKT_INDEX_DATA + (ch*2)]     = adcRes & 0xFF;	// LSB
		pktBuf[PKT_INDEX_DATA + (ch*2) + 1] = adcRes >> 8;	    // MSB
	}

	P1 = 0;		// set MUX input to ground


	// measure VDD
	ADC0CF &= ~ADC0CF_ADGN__GAIN_1;	// set gain to 0.5
	ADC0MX = ADC0MX_ADC0MX__VDD;	// select vdd channel
	adcRes = (((uint32_t)getADC() * cal_fullscale[6]) / 65536) + cal_offset[6];	// apply calibration
	pktBuf[PKT_INDEX_DATA + 12] = adcRes & 0xFF;	// LSB
	pktBuf[PKT_INDEX_DATA + 13] = adcRes >> 8;	    // MSB

	// measure temperature
	REF0CN |= REF0CN_TEMPE__TEMP_ENABLED;	// enable temp sensor
	ADC0MX = ADC0MX_ADC0MX__TEMP;	// select temp channel
	ADC0CF |= ADC0CF_ADGN__GAIN_1;	// set gain to 1.0
	adcRes = (((uint32_t)getADC() * cal_fullscale[7]) / 65536) + cal_offset[7];	// apply calibration
	ADC0MX = ADC0MX_ADC0MX__GND;	// select GND
	REF0CN &= ~REF0CN_TEMPE__TEMP_ENABLED;	// disable temp sensor
	pktBuf[PKT_INDEX_DATA + 14] = adcRes & 0xFF;	// LSB
	pktBuf[PKT_INDEX_DATA + 15] = adcRes >> 8;	    // MSB
}


bool checkParams()
{
	uint8_t i;

	if (nvs_flash[CAL_PAGE_OFFSET_VER] != PARAMS_VER)
		return false;	// wrong version

	if (nvs_flash[CAL_PAGE_OFFSET_VER_INV] != (PARAMS_VER^0xFF))
		return false;	// wrong version complement

	CRC0CN0 |= CRC0CN0_CRCVAL__SET_ONES;
	CRC0CN0 |= CRC0CN0_CRCINIT__INIT;


	for (i=CAL_PAGE_OFFSET_DATA; i<(PARAMS_LEN+CAL_PAGE_OFFSET_DATA); i++)
	{
		CRC0IN = nvs_flash[i];
	}

	CRC0CN0 |= CRC0CN0_CRCPNT__ACCESS_UPPER;	// MSB
	if (CRC0DAT != nvs_flash[CAL_PAGE_OFFSET_CRCH])
		return false;	// CRC error

	CRC0CN0 &= ~CRC0CN0_CRCPNT__ACCESS_UPPER;	// LSB
	if (CRC0DAT != nvs_flash[CAL_PAGE_OFFSET_CRCL])
		return false;	// CRC error

	return true;
}

void loadParams()
{
	uint8_t i;

	if (checkParams())
	{
		// parameters OK, load addr+cal
		ownAddr = nvs_flash[CAL_PAGE_OFFSET_DATA+CAL_DATA_ADDR];

		for (i=0; i<16; i++)
		{
			((uint8_t*)cal_offset)[i] = nvs_flash[CAL_PAGE_OFFSET_DATA+CAL_DATA_OFFSET+i];
			((uint8_t*)cal_fullscale)[i] = nvs_flash[CAL_PAGE_OFFSET_DATA+CAL_DATA_FULLSCALE+i];
		}
	}
}

void flashWriteByte(uint16_t addr, uint8_t byte)
{
  uint8_t SI_SEG_XDATA * pwrite = (uint8_t SI_SEG_XDATA *) addr;

  // Unlock flash by writing the key sequence
  FLKEY = (flash_key1^0xFF);
  FLKEY = (flash_key2^0xFF);

  // Enable flash writes, then do the write
  PSCTL |= PSCTL_PSWE__WRITE_ENABLED;
  *pwrite = byte;
  PSCTL &= ~(PSCTL_PSEE__ERASE_ENABLED|PSCTL_PSWE__WRITE_ENABLED);
}


bool verifyCal()
{
	uint8_t i;

	// --- build CRC ---

	// init CRC
	CRC0CN0 |= CRC0CN0_CRCVAL__SET_ONES;
	CRC0CN0 |= CRC0CN0_CRCINIT__INIT;

	// address byte
	CRC0IN = ownAddr;

	// cal data
	for (i=0; i<16; i++)
	{
		CRC0IN = ((uint8_t*)cal_offset)[i];
	}

	for (i=0; i<16; i++)
	{
		CRC0IN = ((uint8_t*)cal_fullscale)[i];
	}

	// --- verify ---
	CRC0CN0 |= CRC0CN0_CRCPNT__ACCESS_UPPER;	// MSB
	if (CRC0DAT != nvs_flash[CAL_PAGE_OFFSET_CRCH])
		return false;
	CRC0CN0 &= ~CRC0CN0_CRCPNT__ACCESS_UPPER;	// LSB
	if (CRC0DAT != nvs_flash[CAL_PAGE_OFFSET_CRCL])
		return false;

	if ((PARAMS_VER^0xFF) != nvs_flash[CAL_PAGE_OFFSET_VER_INV])
		return false;
	if (PARAMS_VER != nvs_flash[CAL_PAGE_OFFSET_VER])
		return false;

	if (ownAddr != nvs_flash[CAL_PAGE_OFFSET_DATA+CAL_DATA_ADDR])
		return false;

	for (i=0; i<16; i++)
	{
		if (((uint8_t*)cal_offset)[i] != nvs_flash[CAL_PAGE_OFFSET_DATA+CAL_DATA_OFFSET+i])
			return false;
		if (((uint8_t*)cal_fullscale)[i] != nvs_flash[CAL_PAGE_OFFSET_DATA+CAL_DATA_FULLSCALE+i])
			return false;
	}

	return true;
}


bool writeCal()
{
	uint8_t i;


	// check key codes
	if ((0x5A != flash_key1) || (0x0E != flash_key2))
	{
		return false;	// invalid keys
	}

	// first check if update is needed
	if (verifyCal())
		return true;	// no update needed

	// data CRC already calculated in verifycal(), no need to do it again

	// erase cal page
	PSCTL |= PSCTL_PSEE__ERASE_ENABLED;
	flashWriteByte(FLASH_ADDR_CAL, 0);

	// write address byte
	flashWriteByte(FLASH_ADDR_CAL+CAL_PAGE_OFFSET_DATA+CAL_DATA_ADDR, ownAddr);

	// write cal data
	for (i=0; i<16; i++)
	{
		flashWriteByte(FLASH_ADDR_CAL+CAL_PAGE_OFFSET_DATA+CAL_DATA_OFFSET+i, ((uint8_t*)cal_offset)[i]);
		flashWriteByte(FLASH_ADDR_CAL+CAL_PAGE_OFFSET_DATA+CAL_DATA_FULLSCALE+i, ((uint8_t*)cal_fullscale)[i]);
	}

	// write crc
	CRC0CN0 |= CRC0CN0_CRCPNT__ACCESS_UPPER;	// MSB
	flashWriteByte(FLASH_ADDR_CAL+CAL_PAGE_OFFSET_CRCH, CRC0DAT);
	CRC0CN0 &= ~CRC0CN0_CRCPNT__ACCESS_UPPER;	// LSB
	flashWriteByte(FLASH_ADDR_CAL+CAL_PAGE_OFFSET_CRCL, CRC0DAT);

	// write version info
	flashWriteByte(FLASH_ADDR_CAL+CAL_PAGE_OFFSET_VER_INV, (PARAMS_VER^0xFF));
	flashWriteByte(FLASH_ADDR_CAL+CAL_PAGE_OFFSET_VER, PARAMS_VER);

	// verify
	if (!verifyCal())
		return false;	// verify failed

	return true;	// all OK
}

void processPkt()
{
	static bool seqInit = false;
	static uint8_t lastSeq;
	uint8_t dstAddr;
	uint8_t seq;
	uint8_t tmp8;

	if (!cobs_decode(pktBuf_len, pktBuf))
	{
		// COBS decoding failure, ignore packet
		return;
	}

	dstAddr = pktBuf[PKT_INDEX_DST];
	seq = pktBuf[PKT_INDEX_SEQ];

	if ((seqInit) && (seq == lastSeq))
	{
		// already processed this packet
		return;
	}

	seqInit = true;
	lastSeq = seq;

	if ((ownAddr == dstAddr) && (ownAddr != 0))
	{
		// addressed to this node

		// prepare reply
		pktBuf[PKT_INDEX_DST] = pktBuf[PKT_INDEX_SRC];
		pktBuf[PKT_INDEX_SRC] = ownAddr;
		pktBuf[PKT_INDEX_FLAGS] = PKT_FLAG_REPLY;
		pktBuf[PKT_INDEX_LEN] = PKT_MINLEN;


		switch(pktBuf[PKT_INDEX_CMD])
		{
			case PKT_CMD_ID:

				for (tmp8 = 0; tmp8 < 4; tmp8++)
				{
					pktBuf[PKT_INDEX_DATA + tmp8] = uuid[tmp8];
				}

				pktBuf[PKT_INDEX_DATA + 4] = DEVICEID;
				pktBuf[PKT_INDEX_DATA + 5] = DERIVID;
				pktBuf[PKT_INDEX_DATA + 6] = REVID;

				pktBuf[PKT_INDEX_FLAGS] |= PKT_FLAG_OK;
				pktBuf[PKT_INDEX_LEN] += 7;	// 7 bytes payload
				break;
			case PKT_CMD_MEAS:
				doMeas();
				pktBuf[PKT_INDEX_FLAGS] |= PKT_FLAG_OK;
				pktBuf[PKT_INDEX_LEN] += 16;	// 16 bytes payload
				break;

			case PKT_CMD_GETCAL:

				for (tmp8=0; tmp8<16; tmp8++)
				{
					pktBuf[PKT_INDEX_DATA + tmp8] = ((uint8_t*)cal_offset)[tmp8];
					pktBuf[PKT_INDEX_DATA + 16 + tmp8] = ((uint8_t*)cal_fullscale)[tmp8];
				}
				pktBuf[PKT_INDEX_FLAGS] |= PKT_FLAG_OK;
				pktBuf[PKT_INDEX_LEN] += 32;	// 32 bytes payload
				break;

			case PKT_CMD_SETCAL:

				for (tmp8=0; tmp8<16; tmp8++)
				{
					((uint8_t*)cal_offset)[tmp8] = pktBuf[PKT_INDEX_DATA + tmp8];
					((uint8_t*)cal_fullscale)[tmp8] = pktBuf[PKT_INDEX_DATA + 16 + tmp8];
				}
				pktBuf[PKT_INDEX_FLAGS] |= PKT_FLAG_OK;
				break;

			case PKT_CMD_WRITECAL:
				// store received keys
				flash_key1 = pktBuf[PKT_INDEX_DATA];
				flash_key2 = pktBuf[PKT_INDEX_DATA+1];
				// clear key codes from buffer
				pktBuf[PKT_INDEX_DATA] = 0;
				pktBuf[PKT_INDEX_DATA+1] = 0;

				if (writeCal())
				{
					// write OK
					pktBuf[PKT_INDEX_FLAGS] |= PKT_FLAG_OK;
				}

				// clear key codes from memory
				flash_key1 = 0;
				flash_key2 = 0;
				break;

		}

		// send reply
		sendReply();
	}
	else if (dstAddr == 255)
	{
		if ((pktBuf[PKT_INDEX_CMD] == PKT_CMD_SETADDR) && (pktBuf[PKT_INDEX_LEN] == 9))
		{
			// set address
			// check if jumper is present
			//if (!P2_B0)
			if (1)
			{
				// yes, proceed

				// prepare reply
				pktBuf[PKT_INDEX_DST] = pktBuf[PKT_INDEX_SRC];
				pktBuf[PKT_INDEX_FLAGS] = PKT_FLAG_REPLY | PKT_FLAG_OK;
				pktBuf[PKT_INDEX_LEN] = PKT_MINLEN;

				// set address
				ownAddr = pktBuf[PKT_INDEX_DATA];

				// in this case, send reply
				pktBuf[PKT_INDEX_SRC] = ownAddr;
				sendReply();
			}


		}

		// broadcast packet, no reply
	}


}

int main(void)
{
	uint8_t rxByte;


	// Enable voltage monitor
	VDM0CN |= VDM0CN_VDMEN__ENABLED;
	RSTSRC = RSTSRC_PORSF__BMASK;


	// Configure clock
	//CLKSEL = CLKSEL_CLKSL__HFOSC | CLKSEL_CLKDIV__SYSCLK_DIV_128;
	CLKSEL = CLKSEL_CLKSL__HFOSC | CLKSEL_CLKDIV__SYSCLK_DIV_16;

	// Enable LFOSC
	LFO0CN |= LFO0CN_OSCLEN__ENABLED | LFO0CN_OSCLD__DIVIDE_BY_2;

	// Configure Port 0
	P0 = P0_B0__LOW  | P0_B1__LOW  | P0_B2__LOW   | P0_B3__LOW
	   | P0_B4__HIGH | P0_B5__HIGH | P0_B6__HIGH  | P0_B7__HIGH;

	P0MDOUT = P0MDOUT_B0__OPEN_DRAIN | P0MDOUT_B1__OPEN_DRAIN
			| P0MDOUT_B2__OPEN_DRAIN | P0MDOUT_B3__OPEN_DRAIN
			| P0MDOUT_B4__PUSH_PULL  | P0MDOUT_B5__OPEN_DRAIN
			| P0MDOUT_B6__PUSH_PULL  | P0MDOUT_B7__OPEN_DRAIN;

	P0MDIN = P0MDIN_B0__DIGITAL | P0MDIN_B1__DIGITAL
		   | P0MDIN_B2__DIGITAL | P0MDIN_B3__DIGITAL
		   | P0MDIN_B4__DIGITAL | P0MDIN_B5__DIGITAL
		   | P0MDIN_B6__DIGITAL | P0MDIN_B7__ANALOG;

	P0SKIP = P0SKIP_B0__NOT_SKIPPED | P0SKIP_B1__NOT_SKIPPED
		   | P0SKIP_B2__NOT_SKIPPED | P0SKIP_B3__NOT_SKIPPED
		   | P0SKIP_B4__NOT_SKIPPED | P0SKIP_B5__NOT_SKIPPED
		   | P0SKIP_B6__NOT_SKIPPED | P0SKIP_B7__SKIPPED;

	// Configure Port 1
	P1 = P1_B0__LOW | P1_B1__LOW | P1_B2__LOW | P1_B3__LOW;

	P1MDOUT = P1MDOUT_B0__PUSH_PULL | P1MDOUT_B1__PUSH_PULL
			| P1MDOUT_B2__PUSH_PULL | P1MDOUT_B3__OPEN_DRAIN;

	// Configure Crossbar
	XBR0 = XBR0_URT0E__ENABLED;
	XBR2 = XBR2_XBARE__ENABLED;

	// configure ADC
	ADC0CN1 = ADC0CN1_ADCMBE__CM_BUFFER_ENABLED;
	ADC0MX = ADC0MX_ADC0MX__GND;
	ADC0CF = (0x01 << ADC0CF_ADSC__SHIFT) | ADC0CF_ADGN__GAIN_1;
	ADC0AC = ADC0AC_AD12BE__12_BIT_ENABLED | ADC0AC_ADRPT__ACC_64;
	ADC0PWR = ADC0PWR_ADBIAS__MODE1 | 0x04;
	ADC0TK = 40;
	ADC0CN0 = ADC0CN0_ADBMEN__BURST_ENABLED | ADC0CN0_ADCM__TIMER2;	// Burst mode enabled, triggered by Timer2
	REF0CN = REF0CN_IREFLVL__2P4 | REF0CN_REFSL__INTERNAL_VREF;


	// CKCON setup
	CKCON0 = CKCON0_T2ML__SYSCLK | CKCON0_SCA__SYSCLK_DIV_4;

	// Common Timer0/1 setup
	TMOD = TMOD_T0M__MODE1 | TMOD_T1M__MODE2;

	// Timer1 setup (4.785 kHz / 2392 baud)
	TH1 = 176;	// HFOSC / 16 (SCA /4)
	TL1 = 176;	// HFOSC / 16 (SCA /4)

	// Timer1 setup (9.570 kHz / 4785 baud)
	//TH1 = 0xEC;	// HFOSC / 128
	//TH1 = 96;	// HFOSC / 16 (SCA /48)
	TCON_TR1 = 1;

	// setup Timer2 (ADC)
	TMR2CN0 = 0;
	TMR2RL = 0;

	// setup Timer0 (timeout)
	// ~100 ms at HFOSC / 16
	resetTimeout();

	//TMR3CN0 = 0;
	//TMR3RL = 52776;
	//TMR3 = 52776;	// ~100 ms at HFOSC / 16

	// config INT0 (P0.5)
	IT01CF = IT01CF_IN0PL__ACTIVE_LOW | IT01CF_IN0SL__P0_5;
	TCON_IT0 = 1;	// edge triggered

	// load cal data
	loadParams();

	// enable interrupts
	EIE1 |= EIE1_EADC0__ENABLED;
	IE_EA = 1;

	// start timeout counter
	TCON_TR0 = 1;

	// receive pkt

	//enable UART RX
	SCON0_REN = 1;

	P0_B6 = 0;	// LED off


	for (;;)
	{
		// wait for RX
		while (!SCON0_RI)
		{
			if (TCON_TF0)
			{
				// Timer0 overflowed, go to sleep
				resetTimeout();
				pktBuf_len = 0;
				SCON0_REN = 0;	// disable receiver
				SCON0_RI = 0;
				//TCON_TR1 = 0;	// disable UART clock
				CLKSEL = CLKSEL_CLKSL__LFOSC | CLKSEL_CLKDIV__SYSCLK_DIV_1;	// switch to LFOSC
				TCON_IE0 = 0;	// clear INT0
				IE_EX0 = 1;		// enable INT0

				P0_B6 = 0;	// LED off

				// enter low power mode
				PCON0 |= 0x01;
				PCON0 = PCON0;

				// exiting low power mode

				// RX pin went low, exit low power mode

				// switch to high speed oscillator
				CLKSEL = CLKSEL_CLKSL__HFOSC | CLKSEL_CLKDIV__SYSCLK_DIV_16;

				P0_B6 = 1;	// LED on

				// disable INT0
				IE_EX0 = 0;

				//TCON_TR1 = 1;	// enable UART clock

				// wait for RX pin to go high
				while (!P0_B5);

				// enable UART RX
				SCON0_REN = 1;

				//P0_B6 = 0;	// LED off

				// start timeout counter
				TCON_TR0 = 1;
			}
		}

		// data received, reset and restart timeout counter
		resetTimeout();
		TCON_TR0 = 1;	// start timeout counter

		rxByte = SBUF0;	// read byte from FIFO
		SCON0_RI = 0;	// clear UART RI

		if (rxByte == 0)	// frame start/end received, reset packet handler or process packet depending on number of bytes in buffer
		{
			if (pktBuf_len >= PKT_MINLEN)	// minimum valid packet length is PKT_MINLEN bytes
			{
				// entire frame received
				resetTimeout();	// stop timeout counter
				TCON_TR0 = 1;	// use timeout counter to enforce minimum RX -> TX delay

				// process frame
				processPkt();

				// processing complete, wait for next packet
				pktBuf_len = 0;

				// wait 1 ms before going to sleep to allow UART stop bit to finish transmitting if needed
				// 1 msec at HFOSC/16
				//TH0 = 0xFF;
				//TL0 = 0xE0;
				//TCON_TR0 = 1;	// start timeout counter

				TCON_TF0 = 1;	// go to sleep immediately
				SCON0_RI = 0;	// clear UART RI

			}
			else
			{
				// frame start
				pktBuf_len = 0;
			}
		}
		else
		{
			// limit buffer pointer to avoid overflow in case of noise etc.
			if (pktBuf_len < PKT_MAXLEN)
				pktBuf[pktBuf_len++] = rxByte;
		}




	}





}
