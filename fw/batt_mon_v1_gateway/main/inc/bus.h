/*
 * bus.h
 *
 *  Created on: 7 dec. 2021
 *      Author: Mathias
 */

#ifndef MAIN_INC_BUS_H_
#define MAIN_INC_BUS_H_

#include <stdint.h>
#include <stdbool.h>

#define BUS_UART_NUM UART_NUM_1
#define BUF_SIZE (128)

typedef struct
{
	uint8_t flags;
	uint8_t addr;
	uint8_t seq;
	uint8_t cmd;
	uint8_t datalen;
	uint8_t data[PKT_MAXLEN-PKT_MINLEN];
} bus_pkt_t;

typedef enum
{
	PKT_FLAG_CMD = 0,
	PKT_FLAG_DUP,
	PKT_FLAG_ERR,
	PKT_FLAG_OK,
	PKT_FLAG_DECODEFAIL
} pkt_flag_t;

typedef struct
{
	int16_t cal_offset[8];
	uint16_t cal_fullscale[8];
	uint8_t address;
} cal_data_t;


bool busInit();
bool sendBusCmd(bus_pkt_t *txPkt, bus_pkt_t *rxPkt, int timeout);
bool getCalData(cal_data_t *cal_data);
bool setCalData(cal_data_t *cal_data);
bool setAddr(uint8_t newAddr);
bool getMeas(uint8_t addr, int16_t *meas_data);
bool saveCal(uint8_t addr);

extern SemaphoreHandle_t xNodeFwMutex;
extern SemaphoreHandle_t xNodeCfgMutex;

#endif /* MAIN_INC_BUS_H_ */
