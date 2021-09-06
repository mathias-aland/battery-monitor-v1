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
#define PKT_CMD_START		0x03
#define PKT_CMD_STOP		0x04

#define PKT_CMD_SETADDR		0x10

#define ADC_NUM_SAMPLES		16


#define MUX_CH1	1
#define MUX_CH2	3
#define MUX_CH3	2
#define MUX_CH4	4
#define MUX_CH5	6
#define MUX_CH6	5


SI_LOCATED_VARIABLE_NO_INIT(uuid[4], uint8_t, const SI_SEG_XDATA, 0x00FC);

uint8_t pktBuf_len = 0;
uint8_t xdata pktBuf[PKT_MAXLEN];
uint8_t ownAddr = 1;


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

uint16_t xdata cal_offset[8] = {10,10,10,10,10,10,0,-2656};
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

	// 100 msec at HFOSC/16
	TH0 = 0xF3;
	TL0 = 0x8A;

	// 1 sec
	//TH0 = 0x83;
	//TL0 = 0x63;

	//TH0 = 0;
	//TL0 = 0;


}

void sendPkt()
{
	uint8_t txIdx;

	// enforce minimum RX -> TX time by waiting until Timer0 overflows
	while (!TCON_TF0);
	resetTimeout();

	SCON0_TI = 0;	// reset TI flag

	// send frame start
	SBUF0 = 0;
	while (!SCON0_TI);	// wait for TX to finish
	SCON0_TI = 0;	// reset TI flag

	// send buffer
	for (txIdx = 0; txIdx < pktBuf_len; txIdx++)
	{
		SBUF0 = pktBuf[txIdx];
		while (!SCON0_TI);	// wait for TX to finish
		SCON0_TI = 0;	// reset TI flag
	}

	// transmit completed, send frame end and reset index
	SBUF0 = 0;
	while (!SCON0_TI);	// wait for TX to finish
	SCON0_TI = 0;	// reset TI flag

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
	uint16_t adcRes;
	uint8_t ch;

	// measure and put into packet buffer

	ADC0MX = ADC0MX_ADC0MX__ADC0P7;	// select ADC0P7 (battery measurement)

	for (ch = 0; ch < 6; ch++)
	{
		P1 = chMux[ch];	// select MUX channel
		adcRes = getADC();
		adcRes = (((uint32_t)adcRes * cal_fullscale[ch]) / 65536) + cal_offset[ch];	// apply calibration
		pktBuf[PKT_INDEX_DATA + (ch*2)]     = adcRes & 0xFF;	// LSB
		pktBuf[PKT_INDEX_DATA + (ch*2) + 1] = adcRes >> 8;	    // MSB
	}

	P1 = 0;		// set MUX input to ground


	// measure VDD
	ADC0CF &= ~ADC0CF_ADGN__GAIN_1;	// set gain to 0.5
	ADC0MX = ADC0MX_ADC0MX__VDD;	// select vdd channel
	adcRes = getADC();
	adcRes = (((uint32_t)adcRes * cal_fullscale[6]) / 65536) + cal_offset[6];	// apply calibration
	pktBuf[PKT_INDEX_DATA + 12] = adcRes & 0xFF;	// LSB
	pktBuf[PKT_INDEX_DATA + 13] = adcRes >> 8;	    // MSB

	// measure temperature
	REF0CN |= REF0CN_TEMPE__TEMP_ENABLED;	// enable temp sensor
	ADC0MX = ADC0MX_ADC0MX__TEMP;	// select temp channel
	ADC0CF |= ADC0CF_ADGN__GAIN_1;	// set gain to 1.0
	adcRes = getADC();
	adcRes = (((uint32_t)adcRes * cal_fullscale[7]) / 65536) + cal_offset[7];	// apply calibration
	ADC0MX = ADC0MX_ADC0MX__GND;	// select GND
	REF0CN &= ~REF0CN_TEMPE__TEMP_ENABLED;	// disable temp sensor
	pktBuf[PKT_INDEX_DATA + 14] = adcRes & 0xFF;	// LSB
	pktBuf[PKT_INDEX_DATA + 15] = adcRes >> 8;	    // MSB
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

	if (ownAddr == dstAddr)
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

		}

		// send reply
		pktBuf_len = pktBuf[PKT_INDEX_LEN] + 1;

		if (cobs_encode(pktBuf_len, pktBuf))
		{
			// COBS encoding successful
			sendPkt();
		}








	}
	else if (dstAddr == 255)
	{
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
	CKCON0 = CKCON0_T1M__SYSCLK | CKCON0_T2ML__SYSCLK | CKCON0_T3ML__EXTERNAL_CLOCK | CKCON0_SCA__SYSCLK_DIV_48;

	// Common Timer0/1 setup
	TMOD = TMOD_T0M__MODE1 | TMOD_T1M__MODE2;

	// Timer1 setup (9.570 kHz / 4785 baud)

	//TH1 = 0xEC;	// HFOSC / 128
	TH1 = 96;	// HFOSC / 16
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
