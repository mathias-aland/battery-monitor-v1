/*
 * common.h
 *
 *  Created on: 17 sep. 2021
 *      Author: Mathias
 */

#ifndef MAIN_INC_COMMON_H_
#define MAIN_INC_COMMON_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "lwip/ip4_addr.h"
#include "esp_wifi_types.h"

#define PKT_MINLEN 6
#define PKT_MAXLEN 73

#define PKT_INDEX_COBS	0
#define PKT_INDEX_LEN	1
#define PKT_INDEX_DST	2
#define PKT_INDEX_SEQ	3
#define PKT_INDEX_CMD	4
#define PKT_INDEX_DATA	5


#define PKT_CMD_ID			0x00
#define PKT_CMD_STATUS		0x01
#define PKT_CMD_MEAS		0x02
#define PKT_CMD_SETADDR		0x10
#define PKT_CMD_ENTERBOOT	0x0F
#define PKT_CMD_GETCAL		0x11
#define PKT_CMD_SETCAL		0x12
#define PKT_CMD_WRITECAL	0x13



#define BL_CMD_IDENT		0x30	// Send back info
#define BL_CMD_ERASE		0x31	// Erase flash page
#define BL_CMD_READ			0x32	// Read flash  (64 bytes)
#define BL_CMD_WRITE		0x33	// Write flash (64 bytes)
#define BL_CMD_RESET		0x34	// Soft reset


#define FW_BLOCKS			112		// (8192 - 1024) / 64


#define NODE_FW_SIZE	7168



#define COMM_REQ_MEASURE			0x01
#define COMM_REQ_FWUPD				0x02
#define COMM_REQ_MEASURE_CANC		0x04
#define COMM_REQ_FWUPD_CANC			0x08
#define COMM_REQ_GETCAL				0x10
#define COMM_REQ_SETCAL				0x20
#define COMM_REQ_SETADDR			0x40

#define MAX_NODES	16
#define MAX_GROUPS	8
#define MAX_NAME	16





enum {
	FWUPD_STATUS_NO_DATA,
	FWUPD_STATUS_CRC_ERR,
    FWUPD_STATUS_IDLE,
	FWUPD_STATUS_DONE,
	FWUPD_STATUS_FAIL,
	FWUPD_STATUS_PEND,
	FWUPD_STATUS_BUSY
};

enum {
	FWUPD_STAGE_NONE,
	FWUPD_STAGE_ENTER_BOOT,
	FWUPD_STAGE_ERASE,
	FWUPD_STAGE_WRITE,
	FWUPD_STAGE_VERIFY,
	FWUPD_STAGE_RUN
};


enum {
	FWUPD_TYPE_NONE,
    FWUPD_TYPE_ALL,
	FWUPD_TYPE_FAILSAFE
};

enum {
	GROUP_STATE_DISABLED,
	GROUP_STATE_ENABLED
};

enum {
	SETADDR_NONE,
	SETADDR_PEND,
	SETADDR_OK,
	SETADDR_ERR
};

enum {
	NODE_STATE_DISABLED,
	NODE_STATE_ENABLED
};


typedef struct nodefw_s
{
	volatile uint16_t crc;
	volatile uint16_t crc_calc;
	volatile uint8_t status;
	volatile uint8_t curr_block;
	volatile uint8_t curr_stage;
	volatile uint8_t addr;
	uint8_t fwdata[NODE_FW_SIZE];
} nodefw_t;


typedef struct group_s
{
	uint8_t state;
	char name[MAX_NAME];
} group_t;

typedef struct node_s
{
	uint32_t uid;
	uint16_t fw_ver;
	uint16_t fw_crc;
	int16_t meas_data[8];
	int16_t cal_offset[8];
	uint16_t cal_fullscale[8];
	uint8_t address;
	uint8_t group;
	uint8_t group_pos;
	uint8_t fwupd_fail_block;
	uint8_t fwupd_fail_op;
	uint8_t flag_enabled : 1;
	uint8_t flag_error : 1;
	uint8_t flag_paused : 1;
	uint8_t flag_fwupd_pend: 1;
	uint8_t flag_fwupd_err: 1;
	uint8_t flag_fwupd_ok: 1;
	char name[MAX_NAME];
} node_t;


typedef struct
{
	SemaphoreHandle_t xSemaphore;
	uint8_t address;
	uint8_t status;
} setaddr_t;



typedef struct
{
	uint8_t state;
	char name[MAX_NAME];
} nvs_group_t;


typedef struct
{
	uint8_t address;
	uint8_t group;
	uint8_t group_pos;
	uint8_t state;
	char name[MAX_NAME];
} nvs_node_t;



typedef struct
{
	char tz[64];
	char name[64];
} timezone_t;




typedef struct
{
	ip4_addr_t ip_addr;
	ip4_addr_t ip_subnet;
	ip4_addr_t ip_gw;
	ip4_addr_t ip_dns;
	wifi_config_t wifi_config;
	//char wifi_ssid[32];
	//char wifi_pw[64];
	char sntp_srv[64];
	timezone_t timezone;
	uint16_t updInterval;	// node update interval, should probably be moved somewhere else
} netcfg_t;






extern nodefw_t nodefw;
extern TaskHandle_t h_comm_task;
extern group_t groups[MAX_GROUPS];
extern node_t nodes[MAX_NODES];
extern setaddr_t setaddrData;
extern volatile uint8_t seq;
extern netcfg_t netCfg;
extern TimerHandle_t tmr_node_refresh;
extern TimerHandle_t tmr_reboot;

//#define PKT_FLAG_CMD		0
//#define PKT_FLAG_DUP		1
//#define PKT_FLAG_ERR		2
//#define PKT_FLAG_OK			3

#endif /* MAIN_INC_COMMON_H_ */
