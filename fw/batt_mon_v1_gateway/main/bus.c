/*
 * bus.c
 *
 *  Created on: 7 dec. 2021
 *      Author: Mathias
 */


#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "common.h"
#include "cobs.h"
#include "bus.h"

SemaphoreHandle_t xBusCommMutex;
SemaphoreHandle_t xNodeFwMutex;
SemaphoreHandle_t xNodeCfgMutex;

bool busInit()
{
	xBusCommMutex = xSemaphoreCreateMutex();

	if (xBusCommMutex == NULL)
	{
		// cannot create semaphore, abort execution
		return false;
	}

	xNodeFwMutex = xSemaphoreCreateMutex();

	if (xNodeFwMutex == NULL)
	{
		// cannot create semaphore, abort execution
		return false;
	}

	xNodeCfgMutex = xSemaphoreCreateMutex();

	if (xNodeCfgMutex == NULL)
	{
		// cannot create semaphore, abort execution
		return false;
	}

    // init UART

    //const uart_port_t uart_num = UART_NUM_1;
    uart_config_t uart_config = {
        .baud_rate = 2400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // Configure UART parameters
    uart_param_config(BUS_UART_NUM, &uart_config);

    // RX signal is inverted
    uart_set_line_inverse(BUS_UART_NUM, UART_SIGNAL_RXD_INV);

    // Set UART pins(TX: IO14, RX: IO16, RTS: IO15)
    uart_set_pin(BUS_UART_NUM, 14, 16, 15, UART_PIN_NO_CHANGE);

    // Install UART driver
    uart_driver_install(BUS_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    uart_set_mode(BUS_UART_NUM, UART_MODE_RS485_HALF_DUPLEX);


	return true;
}


bool sendBusCmd(bus_pkt_t *txPkt, bus_pkt_t *rxPkt, int timeout)
{
	uint8_t pktBuf[PKT_MAXLEN+3];
	uint8_t rxByte;
	uint8_t rxBuf_len = 0;
	int res;

	if (txPkt->datalen > (PKT_MAXLEN-PKT_MINLEN))
	{
		// too much data
		return false;
	}

	// try to take mutex
	if( xSemaphoreTake( xBusCommMutex, 2000 / portTICK_PERIOD_MS ) == pdFALSE )	// 2 seconds should be enough
	{
		// cannot take mutex
		return false;
	}

	if (txPkt->datalen)
	{
		memcpy(&pktBuf[PKT_INDEX_DATA+1], txPkt->data, txPkt->datalen);
	}

	pktBuf[0] = 0;											// Frame start
	pktBuf[1] = 0;											// COBS header
	pktBuf[PKT_INDEX_LEN+1] = PKT_MINLEN+txPkt->datalen;	// Packet length
	pktBuf[PKT_INDEX_DST+1] = txPkt->addr;					// Address
	pktBuf[PKT_INDEX_SEQ+1] = txPkt->seq;					// Sequence number
	pktBuf[PKT_INDEX_CMD+1] = txPkt->cmd << 2;				// Command
	pktBuf[PKT_INDEX_DATA+1+txPkt->datalen] = 0;			// CRC MSB
	pktBuf[PKT_INDEX_DATA+2+txPkt->datalen] = 0;			// CRC LSB
	pktBuf[PKT_INDEX_DATA+3+txPkt->datalen] = 0;			// Frame end

	// do COBS encode
	cobs_encode(PKT_MINLEN+txPkt->datalen+1, &pktBuf[1]);	// skip frame start/end

	// send packet
	uart_flush_input(BUS_UART_NUM);
	uart_write_bytes(BUS_UART_NUM, pktBuf, PKT_MINLEN+txPkt->datalen+3);
	uart_wait_tx_idle_polling(BUS_UART_NUM);

	// receive reply

	for (;;)
	{
		res = uart_read_bytes(BUS_UART_NUM, &rxByte, 1, timeout / portTICK_PERIOD_MS);

		if (res != 1)
		{
			// error, abort
			xSemaphoreGive(xBusCommMutex);
			return false;
		}

		if (rxByte == 0)	// frame start/end received, reset packet handler or process packet depending on number of bytes in buffer
		{
			if (rxBuf_len >= (PKT_MINLEN+2))
			{
				// process frame
				break;
			}
			else
			{
				// frame start
				rxBuf_len = 1;
			}
		}
		else
		{
			// limit buffer pointer to avoid overflow in case of noise etc.
			if (rxBuf_len < (PKT_MAXLEN+2))
				pktBuf[rxBuf_len++] = rxByte;
		}

	}

	// process frame
	if (!cobs_decode(rxBuf_len-1, &pktBuf[1]))
	{
		// COBS decoding failure, indicate failure
		xSemaphoreGive(xBusCommMutex);
		return false;
	}

	// decode OK
	rxPkt->flags   = (pkt_flag_t)(pktBuf[PKT_INDEX_CMD+1] & 0x03);	// keep bits 1-0 (flags)
	rxPkt->cmd	   = (pktBuf[PKT_INDEX_CMD+1] & 0xFC) >> 2;			// keep bits 7-2 (cmd), shift to right
	rxPkt->datalen = pktBuf[PKT_INDEX_LEN+1] - PKT_MINLEN;
	rxPkt->addr    = pktBuf[PKT_INDEX_DST+1];
	rxPkt->seq     = pktBuf[PKT_INDEX_SEQ+1];

	if (rxPkt->datalen > 0)
		memcpy(rxPkt->data, &pktBuf[PKT_INDEX_DATA+1], rxPkt->datalen);


	xSemaphoreGive(xBusCommMutex);
	return true;

}



bool setCalData(cal_data_t *cal_data)
{

	// set cal data

	bus_pkt_t txPkt;
	bus_pkt_t rxPkt;

	txPkt.cmd = PKT_CMD_SETCAL;
	txPkt.datalen = 32;
	txPkt.addr = cal_data->address;
	txPkt.seq = seq++;

	// copy cal data to tx buffer
	for (int ch=0; ch<8; ch++)
	{
		// convert from big-endian to host order
		txPkt.data[ch * 2] 	   	= cal_data->cal_offset[ch] >> 8;		// MSB
		txPkt.data[ch * 2 + 1] 	= cal_data->cal_offset[ch] & 0xFF;		// LSB
		txPkt.data[ch * 2 + 16] = cal_data->cal_fullscale[ch] >> 8;		// MSB
		txPkt.data[ch * 2 + 17] = cal_data->cal_fullscale[ch] & 0xFF;	// LSB
	}

	// wait for TX and response
	if (sendBusCmd(&txPkt, &rxPkt, 1000))
	{
		// data received
		if ((cal_data->address == rxPkt.addr) && (PKT_CMD_SETCAL == rxPkt.cmd) && (PKT_FLAG_OK == rxPkt.flags))
		{
			return true;
		}
	}

	return false;
}



bool getCalData(cal_data_t *cal_data)
{
	// get cal data

	bus_pkt_t txPkt;
	bus_pkt_t rxPkt;

	txPkt.cmd = PKT_CMD_GETCAL;
	txPkt.datalen = 0;
	txPkt.addr = cal_data->address;
	txPkt.seq = seq++;

	// wait for TX and response
	if (sendBusCmd(&txPkt, &rxPkt, 1000))
	{
		// data received
		if ((cal_data->address == rxPkt.addr) && (PKT_CMD_GETCAL == rxPkt.cmd) && (PKT_FLAG_OK == rxPkt.flags))
		{
			for (int ch=0; ch<8; ch++)
			{
				cal_data->cal_offset[ch] = rxPkt.data[ch * 2 + 1] | (rxPkt.data[ch * 2] << 8);			// convert from big-endian to host order
				cal_data->cal_fullscale[ch] = rxPkt.data[ch * 2 + 17] | (rxPkt.data[ch * 2 + 16] << 8);	// convert from big-endian to host order
			}

			return true;
		}

	}

	return false;

}



bool saveCal(uint8_t addr)
{
	bus_pkt_t txPkt;
	bus_pkt_t rxPkt;

	// save calibration data to flash
	txPkt.cmd = PKT_CMD_WRITECAL;
	txPkt.datalen = 2;
	txPkt.addr = addr;
	txPkt.seq = seq++;
	txPkt.data[0] = 0x5A;	// key 1
	txPkt.data[1] = 0x0E;	// key 2

	// wait for TX and response
	if (!sendBusCmd(&txPkt, &rxPkt, 1000))
	{
		return false;	// failure
	}

	if ((addr != rxPkt.addr) || (PKT_CMD_WRITECAL != rxPkt.cmd) || (PKT_FLAG_OK != rxPkt.flags))
	{
		return false;	// failure
	}

	// success
	return true;
}


bool setAddr(uint8_t newAddr)
{
	bus_pkt_t txPkt;
	bus_pkt_t rxPkt;

	// set addr
	txPkt.cmd = PKT_CMD_SETADDR;
	txPkt.datalen = 1;
	txPkt.addr = 0;
	txPkt.seq = seq++;
	txPkt.data[0] = newAddr;

	// wait for TX and response
	if (!sendBusCmd(&txPkt, &rxPkt, 1000))
	{
		return false;	// failure
	}

	if ((newAddr != rxPkt.addr) || (PKT_CMD_SETADDR != rxPkt.cmd) || (PKT_FLAG_OK != rxPkt.flags))
	{
		return false;	// failure
	}

	// save address to flash
	txPkt.cmd = PKT_CMD_WRITECAL;
	txPkt.datalen = 2;
	txPkt.addr = newAddr;
	txPkt.seq = seq++;
	txPkt.data[0] = 0x5A;	// key 1
	txPkt.data[1] = 0x0E;	// key 2

	// wait for TX and response
	if (!sendBusCmd(&txPkt, &rxPkt, 1000))
	{
		return false;	// failure
	}

	if ((newAddr != rxPkt.addr) || (PKT_CMD_WRITECAL != rxPkt.cmd) || (PKT_FLAG_OK != rxPkt.flags))
	{
		return false;	// failure
	}

	// success
	return true;
}

// meas_data must have room for 8 elements!
bool getMeas(uint8_t addr, int16_t *meas_data)
{
	bus_pkt_t txPkt;
	bus_pkt_t rxPkt;

	txPkt.cmd = PKT_CMD_MEAS;
	txPkt.datalen = 0;
	txPkt.addr = addr;
	txPkt.seq = seq++;

	// wait for TX and response
	if (!sendBusCmd(&txPkt, &rxPkt, 500))	// response should arrive within 300 ms
	{
		return false;	// failure
	}

	// data received
	if ((addr != rxPkt.addr) || (PKT_CMD_MEAS != rxPkt.cmd) || (PKT_FLAG_OK != rxPkt.flags))
	{
		return false;	// failure
	}

	for (int ch=0; ch<8; ch++)
	{
		meas_data[ch] = rxPkt.data[ch * 2 + 1] | (rxPkt.data[ch * 2] << 8);	// convert from big-endian to host order
	}

	return true;
}


