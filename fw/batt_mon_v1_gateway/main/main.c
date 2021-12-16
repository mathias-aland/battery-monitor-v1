/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <endian.h>
#include <time.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "common.h"
#include "cobs.h"
#include <nvs_flash.h>
#include "esp_wifi.h"
#include <sys/param.h>
#include "esp_rom_crc.h"
#include <cJSON.h>
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "tusb_console.h"
#include "esp_console.h"
#include "linenoise/linenoise.h"
#include "argtable3/argtable3.h"
#include "soc/gpio_sig_map.h" //for DTRN pin configuration
#include "esp_rom_gpio.h"
#include "esp_sntp.h"
#include "lwip/def.h"
#include "lwip/dns.h"
#include "bus.h"
#include "websrv.h"

//static QueueHandle_t uart0_queue;

static const char *TAG = "uart_events";





// global packet sequence counter
volatile uint8_t seq = 0;






//typedef enum
//{
//	COMM_REQ_CANCEL = 0,
//	COMM_REQ_MEAS,
//	COMM_REQ_FWUPD
//} comm_reqtype_t;


typedef struct
{
	pkt_flag_t flags;
	uint8_t addr;
	uint8_t seq;
	uint8_t cmd;
	uint8_t datalen;
	uint8_t data[PKT_MAXLEN-6];
} node_response_t;



//typedef struct
//{
//	comm_reqtype_t reqType;
//	int dat_len;
//	uint8_t addr;
//	uint8_t *dat;
//} comm_req_t;

//QueueHandle_t node_response_queue;
//QueueHandle_t comm_req_queue;



enum {
    NODE_STATUS_UNKNOWN,
    NODE_STATUS_OK,
    NODE_STATUS_UNREACHABLE,
	NODE_STATUS_DISABLED
};









enum {
	CAL_NONE,
	CAL_PEND,
	CAL_OK,
	CAL_ERR
};

enum {
	CAL_OP_NONE,
	CAL_OP_GET,
	CAL_OP_SET,
	CAL_OP_SAVE
};











#define LED_CMD_CONNECTED		0x01
#define LED_CMD_DISCONNECTED	0x02

#define LED_CMD_MODE_NORMAL		0x04
#define LED_CMD_MODE_AP			0x08









typedef struct
{
	SemaphoreHandle_t xSemaphore;
	int16_t cal_offset[8];
	uint16_t cal_fullscale[8];
	uint8_t address;
	uint8_t status;
	uint8_t cal_op;
} caldata_t;

netcfg_t netCfg;
group_t groups[MAX_GROUPS] = {0};
node_t nodes[MAX_NODES] = {0};
node_t failsafe_node = {
		.uid = 0,
		.address = 0
};

nodefw_t nodefw;

caldata_t caldata;

setaddr_t setaddrData = {
		.xSemaphore = NULL,
		.status = SETADDR_NONE,
		.address = 0
};


TaskHandle_t h_comm_task;
TaskHandle_t h_led_task;
TaskHandle_t h_button_task;

TimerHandle_t tmr_node_refresh;
TimerHandle_t tmr_reboot;

TimerHandle_t led1_timer;
TimerHandle_t led2_timer;
TimerHandle_t led3_timer;


// mutexes






static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
//    httpd_handle_t* server = (httpd_handle_t*) arg;
//    if (*server) {
//        ESP_LOGI(TAG, "Stopping webserver");
//        stop_webserver(*server);
//        *server = NULL;
//    }
//
//    if (sntp_enabled())
//    	sntp_stop();

    xTaskNotify(h_led_task, LED_CMD_DISCONNECTED, eSetBits);
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
//    httpd_handle_t* server = (httpd_handle_t*) arg;
//    if (*server == NULL) {
//        ESP_LOGI(TAG, "Starting webserver");
//        *server = start_webserver();
//    }
//
//
//    if (sntp_enabled())
//    	sntp_stop();
//
//    sntp_init();


    xTaskNotify(h_led_task, LED_CMD_CONNECTED, eSetBits);
}






//bool processPkt(uint8_t *buf, uint8_t len)
//{
//	// allocate memory for response
//	node_response_t* resp = (node_response_t*)malloc(sizeof(node_response_t));
//
//	if (resp == NULL)
//		return false;	// memory allocation failed
//
//	// clear data
//	memset(resp, 0, sizeof(node_response_t));
//
//	if (!cobs_decode(len, buf))
//	{
//		// COBS decoding failure, indicate failure
//		resp->flags = PKT_FLAG_DECODEFAIL;
//	}
//	else
//	{
//		// decode OK
//		resp->flags = (pkt_flag_t)(buf[PKT_INDEX_CMD] & 0x03);	// keep bits 1-0 (flags)
//		resp->cmd	= (buf[PKT_INDEX_CMD] & 0xFC) >> 2;	// keep bits 7-2 (cmd), shift to right
//		resp->datalen = buf[PKT_INDEX_LEN] - PKT_MINLEN;
//		resp->addr = buf[PKT_INDEX_DST];
//		resp->seq = buf[PKT_INDEX_SEQ];
//
//		if (resp->datalen > 0)
//			memcpy(resp->data, &buf[PKT_INDEX_DATA], resp->datalen);
//
//	}
//
//	// send to queue
//
//	if (xQueueSend(node_response_queue, &resp, 0) != pdTRUE)
//	{
//		// failed, free memory used by response
//		free(resp);
//		return false;
//	}
//
//	return true;
//}
//
//
//static void uart_event_task(void *pvParameters)
//{
//    uart_event_t event;
//    //size_t buffered_size;
//    //uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
//    uint8_t dtmp[RD_BUF_SIZE];
//	uint8_t rxBuf[PKT_MAXLEN];
//	uint8_t rxBuf_len = 0;
//
//    for(;;) {
//        //Waiting for UART event.
//        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
//            bzero(dtmp, RD_BUF_SIZE);
//            //ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
//            switch(event.type) {
//                //Event of UART receving data
//                /*We'd better handler data event fast, there would be much more data events than
//                other types of events. If we take too much time on data event, the queue might
//                be full.*/
//                case UART_DATA:
//                    //printf("[UART DATA]: %d\r\n", event.size);
//                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
//
//                    for (int i=0; i<event.size; i++)
//                    {
//                    	uint8_t rxByte =  dtmp[i];
//
//                    	//printf("%02X",rxByte);
//
//                    	// handle RX data
//
//                    	if (rxByte == 0)	// frame start/end received, reset packet handler or process packet depending on number of bytes in buffer
//                    	{
//                    		if (rxBuf_len >= PKT_MINLEN)
//                    		{
//                    			// process frame
//
//								processPkt(rxBuf, rxBuf_len);
//
//								// processing complete, wait for next packet
//                    			rxBuf_len = 0;
//                    		}
//                    		else
//                    		{
//                    			// frame start
//                    			rxBuf_len = 0;
//                    		}
//                    	}
//                    	else
//                    	{
//							// limit buffer pointer to avoid overflow in case of noise etc.
//							if (rxBuf_len < PKT_MAXLEN)
//								rxBuf[rxBuf_len++] = rxByte;
//                    	}
//
//
//
//
//                    }
//
//                    //printf("\r\n");
//
//
//                    //ESP_LOGI(TAG, "[DATA EVT]:");
//                    //uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);
//                    break;
//                //Event of HW FIFO overflow detected
//                case UART_FIFO_OVF:
//                    ESP_LOGI(TAG, "hw fifo overflow");
//                    // If fifo overflow happened, you should consider adding flow control for your application.
//                    // The ISR has already reset the rx FIFO,
//                    // As an example, we directly flush the rx buffer here in order to read more data.
//                    uart_flush_input(EX_UART_NUM);
//                    xQueueReset(uart0_queue);
//                    rxBuf_len = 0;
//                    break;
//                //Event of UART ring buffer full
//                case UART_BUFFER_FULL:
//                    ESP_LOGI(TAG, "ring buffer full");
//                    // If buffer full happened, you should consider encreasing your buffer size
//                    // As an example, we directly flush the rx buffer here in order to read more data.
//                    uart_flush_input(EX_UART_NUM);
//                    xQueueReset(uart0_queue);
//                    rxBuf_len = 0;
//                    break;
//                //Event of UART RX break detected
//                case UART_BREAK:
//                    ESP_LOGI(TAG, "uart rx break");
//                    rxBuf_len = 0;
//                    break;
//                //Event of UART frame error
//                case UART_FRAME_ERR:
//                    ESP_LOGI(TAG, "uart frame error");
//                    rxBuf_len = 0;
//                    break;
//                //Others
//                default:
//                    ESP_LOGI(TAG, "uart event type: %d", event.type);
//                    break;
//            }
//        }
//    }
//    vTaskDelete(NULL);
//}



//
//void doMeas(comm_req_t *commReq)
//{
//
//    uint8_t txBuf[PKT_MAXLEN];
//    node_response_t* resp;
//
//	// send test packet
//	txBuf[1] = 8;			// pkt len
//	txBuf[2] = commReq->addr;			// dst addr
//	txBuf[3] = seq++;   // seq
//	txBuf[4] = (0x02 << 2);	// measure cmd
//
//
//	cobs_encode(txBuf[1]+1, txBuf);
//
//	uint8_t zeroByte[] = {0};
//
//	uart_write_bytes(EX_UART_NUM, zeroByte, 1);
//	uart_write_bytes(EX_UART_NUM, txBuf, txBuf[1]+1);
//	uart_write_bytes(EX_UART_NUM, zeroByte, 1);
//
//	// wait for TX and response
//	if (xQueueReceive(node_response_queue, &resp, 3000 / portTICK_PERIOD_MS) == pdTRUE)
//	{
//		// data received
//
//		printf("addr: %u ", resp->addr);
//		printf("cmd: %u ", resp->cmd);
//		printf("dlen: %u ", resp->datalen);
//		printf("flags: %u ", resp->flags);
//		printf("seq: %u\r\n\r\n", resp->seq);
//
//
//		free(resp);
//	}
//}















//bool node_fwupd(node_t *pNode, uint8_t *txBuf)
bool node_fwupd()
{
	bus_pkt_t txPkt;
	bus_pkt_t rxPkt;

    //node_response_t* resp;
    bool success;

     // try to take mutex
	if( xSemaphoreTake( xNodeFwMutex, 2000 / portTICK_PERIOD_MS ) == pdFALSE )
	{
		// cannot take mutex
		printf("--- Failed to acquire mutex, update aborted ---\n");
		return false;
	}

	if (nodefw.status != FWUPD_STATUS_PEND)
	{
		printf("--- Update status is not PEND, aborting! ---\n");
		xSemaphoreGive(xNodeFwMutex);
		return false;
	}

	printf("--- Start FW update on node 0x%02X ---\n", nodefw.addr);

    nodefw.status = FWUPD_STATUS_BUSY;
	nodefw.curr_block = 0;

	// try to enter bootloader mode and verify mode switch
	nodefw.curr_stage = FWUPD_STAGE_ENTER_BOOT;
	printf("Enter bootloader\n");

	txPkt.datalen = 0;
	txPkt.addr = nodefw.addr;

	success = false;

	for (int retryCnt = 0; retryCnt < 10; retryCnt++ )
	{
		// try to enter bootloader mode
		txPkt.seq  = seq++;
		txPkt.cmd = PKT_CMD_ENTERBOOT;		// enter boot cmd

		sendBusCmd(&txPkt, &rxPkt, 1000);	// response not checked, cmd might fail if already in bootloader

		vTaskDelay(500 / portTICK_PERIOD_MS);	// give bootloader some time to initialize

		// make sure sequence counter is not zero, otherwise the newly initialized bootloader will flag the pkt as duplicate
		if (seq == 0)
			seq++;

		txPkt.seq = seq++;
		txPkt.cmd = BL_CMD_IDENT;	// bootloader identify cmd


		// wait for TX and response
		if (sendBusCmd(&txPkt, &rxPkt, 1000))
		{
			// data received
			if ((nodefw.addr == rxPkt.addr) && (BL_CMD_IDENT == rxPkt.cmd) && (PKT_FLAG_OK == rxPkt.flags))
			{
				// bootloader running
				success = true;
				break;	// exit loop
			}
		}
	}

	if (!success)
	{
		// failure
		printf("Enter bootloader ERR\n");
		nodefw.status = FWUPD_STATUS_FAIL;
		xSemaphoreGive(xNodeFwMutex);
		return false;
	}

	// erase flash
	nodefw.curr_stage = FWUPD_STAGE_ERASE;
	printf("Erase flash\n");


	txPkt.cmd = BL_CMD_ERASE;		// boot erase flash cmd
	txPkt.datalen = 3;
	txPkt.data[1] = 0x5A;	// key 1
	txPkt.data[2] = 0x0E;	// key 2

	for (int blk = 0; blk < FW_BLOCKS; blk += 8)
	{
		nodefw.curr_block = blk;
		txPkt.data[0] = blk;
		txPkt.seq  = seq++;

		success = false;
		// 10 retries should be enough
		for (int retryCnt = 0; retryCnt < 10; retryCnt++ )
		{
			// wait for TX and response
			if (sendBusCmd(&txPkt, &rxPkt, 1000))
			{
				// response received
				if ((nodefw.addr == rxPkt.addr) && (BL_CMD_ERASE == rxPkt.cmd) && ((PKT_FLAG_OK == rxPkt.flags) || (PKT_FLAG_DUP == rxPkt.flags)))	// duplicate counts as OK in this case
				{
					success = true;
					break;	// exit loop
				}
			}
		}

		if (!success)
		{
			// failure
			printf("Erase flash ERR\n");
			nodefw.status = FWUPD_STATUS_FAIL;
			xSemaphoreGive(xNodeFwMutex);
			return false;
		}
	}

	// write flash
	nodefw.curr_block = 0;
	nodefw.curr_stage = FWUPD_STAGE_WRITE;
	printf("Write flash\n");

	txPkt.cmd = BL_CMD_WRITE;	// boot write flash cmd
	txPkt.datalen = 67;
	txPkt.data[1] = 0x5A;		// key 1
	txPkt.data[2] = 0x0E;		// key 2

	for (int blk = 0; blk < FW_BLOCKS; blk++)
	{
		nodefw.curr_block = blk;

		// check if write is needed (not FF)

		bool writeNeeded = false;

		for (int i = 0; i < 64 ; i++)
		{
			if (nodefw.fwdata[blk * 64 + i] != 0xFF)
			{
				// not blank
				writeNeeded = true;
				break;
			}
		}

		if (!writeNeeded)
			continue;


		txPkt.data[0] = blk;
		txPkt.seq  = seq++;

		// copy FW data to buffer
		memcpy(&txPkt.data[3], &nodefw.fwdata[blk * 64], 64);

		success = false;
		// 10 retries should be enough
		for (int retryCnt = 0; retryCnt < 10; retryCnt++ )
		{

			// wait for TX and response
			if (sendBusCmd(&txPkt, &rxPkt, 1000))
			{
				// response received
				if ((nodefw.addr == rxPkt.addr) && (BL_CMD_WRITE == rxPkt.cmd) && ((PKT_FLAG_OK == rxPkt.flags) || (PKT_FLAG_DUP == rxPkt.flags)))	// duplicate counts as OK in this case
				{
					success = true;
					break;	// exit loop
				}
			}
		}

		if (!success)
		{
			// failure
			printf("Write flash ERR\n");
			nodefw.status = FWUPD_STATUS_FAIL;
			xSemaphoreGive(xNodeFwMutex);
			return false;
		}
	}

	// verify flash
	nodefw.curr_block = 0;
	nodefw.curr_stage = FWUPD_STAGE_VERIFY;
	printf("Verify flash\n");



	txPkt.cmd = BL_CMD_READ;	// boot read flash cmd
	txPkt.datalen = 1;

	for (int blk = 0; blk < FW_BLOCKS; blk++)
	{
		nodefw.curr_block = blk;
		txPkt.data[0] = blk;

		success = false;
		// 10 retries should be enough
		for (int retryCnt = 0; retryCnt < 10; retryCnt++ )
		{

			txPkt.seq = seq++;

			// wait for TX and response
			if (sendBusCmd(&txPkt, &rxPkt, 1000))
			{
				// response received
				if ((nodefw.addr == rxPkt.addr) && (BL_CMD_READ == rxPkt.cmd) && (PKT_FLAG_OK == rxPkt.flags))
				{
					// check data
					bool verifyFail = false;

					for (int i = 0; i < 64 ; i++)
					{
						if (nodefw.fwdata[blk * 64 + i] != rxPkt.data[1+i])
						{
							// mismatch
							verifyFail = true;
							break;
						}
					}

					if (!verifyFail)
					{
						success = true;
						break;	// exit loop
					}
				}
			}
		}

		if (!success)
		{
			// failure
			printf("Verify flash ERR\n");
			nodefw.status = FWUPD_STATUS_FAIL;
			xSemaphoreGive(xNodeFwMutex);
			return false;
		}
	}



	// try to exit bootloader mode and verify mode switch
	nodefw.curr_block = 0;
	nodefw.curr_stage = FWUPD_STAGE_RUN;
	printf("Exit bootloader\n");

	txPkt.datalen = 0;

	success = false;

	for (int retryCnt = 0; retryCnt < 10; retryCnt++ )
	{
		// try to exit bootloader mode
		txPkt.cmd = BL_CMD_RESET;	// reset cmd
		txPkt.seq = seq++;

		// wait for TX and response
		sendBusCmd(&txPkt, &rxPkt, 1000); // response not checked, OK here means only that the reset cmd is received but app verification can still fail at bootloader startup

		vTaskDelay(1000 / portTICK_PERIOD_MS);	// give bootloader and app some time to initialize

		// make sure sequence counter is not zero, otherwise the newly initialized app might flag the pkt as duplicate
		if (seq == 0)
			seq++;

		txPkt.cmd = PKT_CMD_ID;	// app identify cmd
		txPkt.seq = seq++;

		// wait for TX and response
		if (sendBusCmd(&txPkt, &rxPkt, 1000))
		{
			// data received
			if ((nodefw.addr == rxPkt.addr) && (PKT_CMD_ID == rxPkt.cmd) && (PKT_FLAG_OK == rxPkt.flags))
			{
				// app running
				success = true;
				break;	// exit loop
			}
		}
	}

	if (!success)
	{
		// failure
		printf("Exit bootloader ERR\n");
		nodefw.status = FWUPD_STATUS_FAIL;
		xSemaphoreGive(xNodeFwMutex);
		return false;
	}


	// update successful
	printf("--- Update OK ---\n");
	nodefw.status = FWUPD_STATUS_DONE;
	xSemaphoreGive(xNodeFwMutex);
	return true;
}


//bool node_setaddr(uint8_t newAddr, uint8_t *txBuf)
//{
//	bus_pkt_t txPkt;
//	bus_pkt_t rxPkt;
//
//    //node_response_t* resp;
//	bool success;
//
//
//	// set addr
//	success = false;
//	for (int retryCnt = 0; retryCnt < 3; retryCnt++ )
//	{
//
//
//		txPkt.cmd = PKT_CMD_SETADDR;
//		txPkt.datalen = 1;
//		txPkt.addr = 0;
//		txPkt.seq = seq++;
//		txPkt.data[0] = newAddr;
//
//		// wait for TX and response
//		if (sendBusCmd(&txPkt, &rxPkt, 1000))
//		{
//			// data received
//			if ((newAddr == rxPkt.addr) && (PKT_CMD_SETADDR == rxPkt.cmd) && (PKT_FLAG_OK == rxPkt.flags))
//			{
//				// OK
//				success = true;
//				break;	// exit loop
//			}
//		}
//	}
//
//	if (!success)
//	{
//		// failure
//		printf("Set address ERR\n");
//		return false;
//	}
//
//	// save address to flash
//	success = false;
//	seq++;
//
//
//	txPkt.cmd = PKT_CMD_WRITECAL;
//	txPkt.datalen = 2;
//	txPkt.addr = newAddr;
//	txPkt.data[0] = 0x5A;	// key 1
//	txPkt.data[1] = 0x0E;	// key 2
//
//	for (int retryCnt = 0; retryCnt < 3; retryCnt++ )
//	{
//		txPkt.seq = seq++;
//
//		// wait for TX and response
//		if (sendBusCmd(&txPkt, &rxPkt, 1000))
//		{
//			// data received
//			if ((newAddr == rxPkt.addr) && (PKT_CMD_WRITECAL == rxPkt.cmd) && (PKT_FLAG_OK == rxPkt.flags))
//			{
//				// OK
//				success = true;
//				break;	// exit loop
//			}
//		}
//	}
//
//	if (!success)
//	{
//		// failure
//		printf("Save address ERR\n");
//		return false;
//	}
//
//	return true;
//}


// node refresh timer callback
static void node_refresh_trig(TimerHandle_t xTimer)
{
	//printf("Meas request\n");
	xTaskNotify(h_comm_task, COMM_REQ_MEASURE, eSetBits);	// trigger measurement collection (no-op if already pending)
}


// COMM task

static void comm_task(void *pvParameters)
{


    //comm_req_t *commReq;


    uint32_t ulNotifiedValue;

    //uint8_t txBuf[PKT_MAXLEN+2]; // make room for leading and trailing 0

   // node_response_t* resp;

	bus_pkt_t txPkt;
	bus_pkt_t rxPkt;


   // int meas_currnode = 0;
    //int fwupd_currnode = 0;
    //bool meas_active = false;
    //bool fwupd_active = false;


    for (;;)
    {

//    	if (meas_active || fwupd_active)
//    	{
//    		// actions pending, check for new events but do not block in this case
//        	if (xTaskNotifyWait(0, ULONG_MAX, &ulNotifiedValue, 0) == pdFALSE) 	// wait for event
//        	{
//        		// timeout
//        		ulNotifiedValue = 0;
//        	}
//    	}
//    	else
//    	{
    		// no actions pending, OK to block until event received
        	xTaskNotifyWait(0, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);	// wait for event
 //   	}


//    	if (ulNotifiedValue & COMM_REQ_SETCAL)
//    	{
//    		// set cal data
//    		caldata.status = CAL_PEND;
//
//    		//caldata.success = false;
//
//    		txPkt.cmd = PKT_CMD_SETCAL;
//    		txPkt.datalen = 32;
//    		txPkt.addr = caldata.address;
//
//			// copy cal data to tx buffer
//			for (int ch=0; ch<8; ch++)
//			{
//				// convert from big-endian to host order
//				txPkt.data[ch * 2] 	   	= caldata.cal_offset[ch] >> 8;		// MSB
//				txPkt.data[ch * 2 + 1] 	= caldata.cal_offset[ch] & 0xFF;	// LSB
//				txPkt.data[ch * 2 + 16] = caldata.cal_fullscale[ch] >> 8;	// MSB
//				txPkt.data[ch * 2 + 17] = caldata.cal_fullscale[ch] & 0xFF;	// LSB
//			}
//
//			int retryCnt = 3;
//
//			do
//			{
//
//				txPkt.seq = seq++;   				// seq
//
//				// wait for TX and response
//				if (sendBusCmd(&txPkt, &rxPkt, 1000))
//				{
//					// data received
//
//					if ((caldata.address == rxPkt.addr) && (PKT_CMD_SETCAL == rxPkt.cmd) && (PKT_FLAG_OK == rxPkt.flags))
//					{
//
//						caldata.status = CAL_OK;
//
//						//caldata.success = true;
//						xSemaphoreGive(caldata.xSemaphore);
//
//						break;	// exit loop
//					}
//
//				}
//
//			} while (--retryCnt);
//    	}
//
//    	if (ulNotifiedValue & COMM_REQ_GETCAL)
//    	{
//    		// get cal data
//
//    		//caldata.success = false;
//    		caldata.status = CAL_PEND;
//
//			int retryCnt = 3;
//
//			do
//			{
//
//
//				txPkt.cmd = PKT_CMD_GETCAL;
//				txPkt.datalen = 0;
//				txPkt.addr = caldata.address;
//				txPkt.seq = seq++;
//
//				// wait for TX and response
//				if (sendBusCmd(&txPkt, &rxPkt, 1000))
//				{
//					// data received
//
//					if ((caldata.address == rxPkt.addr) && (PKT_CMD_GETCAL == rxPkt.cmd) && (PKT_FLAG_OK == rxPkt.flags))
//					{
//
//						for (int ch=0; ch<8; ch++)
//						{
//							caldata.cal_offset[ch] = rxPkt.data[ch * 2 + 1] | (rxPkt.data[ch * 2] << 8);			// convert from big-endian to host order
//							caldata.cal_fullscale[ch] = rxPkt.data[ch * 2 + 17] | (rxPkt.data[ch * 2 + 16] << 8);	// convert from big-endian to host order
//						}
//
//						caldata.status = CAL_OK;
//
//						//caldata.success = true;	// set data valid flag
//						// signal data ready
//						xSemaphoreGive(caldata.xSemaphore);
//
//						break;	// exit loop
//					}
//
//				}
//
//			} while (--retryCnt);
//    	}

//		if (ulNotifiedValue & COMM_REQ_SETADDR)
//		{
//
//
//			if (node_setaddr(setaddrData.address, txBuf))
//			{
//				// OK
//				setaddrData.status = SETADDR_OK;
//			}
//			else
//			{
//				// error
//				setaddrData.status = SETADDR_ERR;
//			}
//
//			xSemaphoreGive(setaddrData.xSemaphore);
//
//		}


    	if (ulNotifiedValue & COMM_REQ_FWUPD)
    	{
    		// perform FW update on node

    		//if ((!fwupd_active) && ((nodefw.status == FWUPD_STATUS_IDLE) || (nodefw.status == FWUPD_STATUS_DONE)))
    		if (nodefw.status == FWUPD_STATUS_PEND)
			{
				// start FW update


    			node_fwupd(&nodefw);

    			nodefw.curr_block = 0;
    			nodefw.curr_stage = FWUPD_STAGE_NONE;


    			//nodefw.curr_addr = 0;
    			//fwupd_currnode = 0;	// start from the beginning
    			//fwupd_active = true;
			}
    	}
//
//    	if (ulNotifiedValue & COMM_REQ_FWUPD_CANC)
//		{
//			// cancel FW update
//			if (fwupd_active)
//			{
//				// cancel FW update
//				fwupd_active = false;
//			}
//		}


    	if (ulNotifiedValue & COMM_REQ_MEASURE)
		{


    		// do measurements

    		printf("--- Start measurement ---\n");


    		for (int i=0; i<MAX_NODES; i++)
    		{
        		// collect measurements
    			if ((nodes[i].address != 0) && (nodes[i].flag_enabled) && (!nodes[i].flag_paused))
    			{
    				// gather data from node
    				int retryCnt = 3;

    				do
    				{
    					txPkt.cmd = PKT_CMD_MEAS;
    					txPkt.datalen = 0;
    					txPkt.addr = nodes[i].address;
    					txPkt.seq = seq++;

    					printf("--- Request measurement from node %i ---\n", txPkt.addr);

    					// wait for TX and response
    					if (sendBusCmd(&txPkt, &rxPkt, 500))	// response should arrive within 300 ms
    					{
    						// data received

    						if ((nodes[i].address == rxPkt.addr) && (PKT_CMD_MEAS == rxPkt.cmd) && (PKT_FLAG_OK == rxPkt.flags))
    						{

    							for (int ch=0; ch<8; ch++)
    							{
    								//pNode->meas_data[ch] = (int16_t)be16toh(((uint16_t*)resp->data)[ch]);

    								nodes[i].meas_data[ch] = rxPkt.data[ch * 2 + 1] | (rxPkt.data[ch * 2] << 8);	// convert from big-endian to host order


    							}


    							printf("OK!\n");
    							//printf("Meas OK at addr %d\n", pNode->address);
    							nodes[i].flag_error = 0;	// clear error flag
    							break;	// exit loop
    						}
    						else
    						{
    							printf("Failed!\n");
    						}

    					}

    				} while (--retryCnt);

    				if (retryCnt == 0)
    				{
    					// failed
    					printf("Timeout!\n");
    					nodes[i].flag_error = 1;	// set error flag
    					//printf("Meas ERR at addr %d\n", pNode->address);
    				}



    			}

    		}






//    		if (!meas_active)
//    		{
//    			// start measurement collection
//    			meas_currnode = 0;	// start from the beginning
//    			meas_active = true;
//    		}
		}

//
//    	if (ulNotifiedValue & COMM_REQ_MEASURE_CANC)
//		{
//			if (meas_active)
//			{
//				// cancel measurement collection
//				meas_active = false;
//			}
//		}



//
//    	if (fwupd_active)
//    	{
//    		// perform FW update on selected nodes
//
//    		if (nodefw.upd_type == FWUPD_TYPE_FAILSAFE)
//    		{
//
//    			failsafe_node.flag_fwupd_err = false;
//    			failsafe_node.flag_fwupd_ok = false;
//				node_fwupd(&failsafe_node);
//				failsafe_node.flag_fwupd_pend = false;
//				fwupd_active = false;
//				fwupd_currnode = 0;
//				nodefw.curr_addr = 0;
//				nodefw.upd_type = FWUPD_TYPE_NONE;
//				nodefw.status = FWUPD_STATUS_DONE;
//    		}
//    		else if (nodefw.upd_type == FWUPD_TYPE_ALL)
//    		{
//    			node_t *pNode = &nodes[fwupd_currnode];	// point to current node in list
//
//
//				//if ((pNode->address != 0) && (pNode->flag_fwupd_pend))
//				if ((pNode->address != 0))
//				{
//					// FW update pending on this node
//					pNode->flag_fwupd_err = false;
//					pNode->flag_fwupd_ok = false;
//					node_fwupd(pNode);
//					pNode->flag_fwupd_pend = false;
//				}
//
//				if (++fwupd_currnode == MAX_NODES)
//				{
//					// done
//					fwupd_active = false;
//					fwupd_currnode = 0;
//					nodefw.curr_addr = 0;
//					nodefw.upd_type = FWUPD_TYPE_NONE;
//					nodefw.status = FWUPD_STATUS_DONE;
//				}
//				else
//				{
//					nodefw.status = FWUPD_STATUS_PEND;
//				}
//    		}
//    		else
//    		{
//    			fwupd_active = false;
//				fwupd_currnode = 0;
//				nodefw.curr_addr = 0;
//				nodefw.upd_type = FWUPD_TYPE_NONE;
//				nodefw.status = FWUPD_STATUS_DONE;
//    		}
//    	}





//    	if (meas_active)
//    	{
//    		// collect measurements
//    		//printf("Meas request ACK\n");
//
//    		node_t *pNode = &nodes[meas_currnode];	// point to current node in list
//
//
//			if ((pNode->address != 0) && (pNode->flag_enabled) && (!pNode->flag_paused))
//			{
//				// gather data from node
//				int retryCnt = 3;
//
//				do
//				{
//					txPkt.cmd = PKT_CMD_MEAS;
//					txPkt.datalen = 0;
//					txPkt.addr = pNode->address;
//					txPkt.seq = seq++;
//
//					// wait for TX and response
//					if (sendBusCmd(&txPkt, &rxPkt, 1000))
//					{
//						// data received
//
//						if ((pNode->address == rxPkt.addr) && (PKT_CMD_MEAS == rxPkt.cmd) && (PKT_FLAG_OK == rxPkt.flags))
//						{
//
//							for (int ch=0; ch<8; ch++)
//							{
//								//pNode->meas_data[ch] = (int16_t)be16toh(((uint16_t*)resp->data)[ch]);
//
//								pNode->meas_data[ch] = rxPkt.data[ch * 2 + 1] | (rxPkt.data[ch * 2] << 8);	// convert from big-endian to host order
//
//
//							}
//
//
//							//printf("Meas OK at addr %d\n", pNode->address);
//							pNode->flag_error = 0;	// clear error flag
//							break;	// exit loop
//						}
//
//					}
//
//				} while (--retryCnt);
//
//				if (retryCnt == 0)
//				{
//					// failed
//					pNode->flag_error = 1;	// set error flag
//					//printf("Meas ERR at addr %d\n", pNode->address);
//				}
//
//
//
//			}
//
//
//			if (++meas_currnode == MAX_NODES)
//			{
//				// done
//				meas_active = false;
//				meas_currnode = 0;
//			}
//
//    	}







    }













//
//    for (;;)
//    {
//
//    	if (xQueueReceive(comm_req_queue, &commReq, portMAX_DELAY) == pdTRUE) {
//    		printf("Meas request recv\n");
//    		// process request
//    		switch (commReq->reqType)
//    		{
//				case COMM_REQ_MEAS:
//					doMeas(commReq);
//					break;
//				default:
//					break;
//    		}
//
//    		// free memory used by request
//    		printf("Meas request free\n");
//
//    		if (commReq->dat != NULL)
//    			free(commReq->dat);
//
//    		free(commReq);
//
//
//    	}
//
//    }

	vTaskDelete(NULL);
}


typedef enum {
	LED_STATE_INIT,
	LED_STATE_DISCONNECTED,
	LED_STATE_CONNECTING,
	LED_STATE_CONNECTED,
	LED_STATE_AP
} ledstate_t;

ledstate_t ledstate = LED_STATE_INIT;



// led1 timer callback
static void led1_elapsed(TimerHandle_t xTimer)
{
	// toggle LED
	static bool lastState = false;
	gpio_set_level(GPIO_NUM_1, lastState);
	lastState = !lastState;
}

// led2 timer callback
static void led2_elapsed(TimerHandle_t xTimer)
{
	// toggle LED
	static bool lastState = false;
	gpio_set_level(GPIO_NUM_2, lastState);
	lastState = !lastState;
}

// led3 timer callback
static void led3_elapsed(TimerHandle_t xTimer)
{
	// toggle LED
	static bool lastState = false;
	gpio_set_level(GPIO_NUM_3, lastState);
	lastState = !lastState;
}



static void led_task(void *pvParameters)
{
	uint32_t ulNotifiedValue;
	//uint32_t currentDelay;
    // configure status leds
    gpio_set_direction(GPIO_NUM_1,GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_2,GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_3,GPIO_MODE_OUTPUT);

    gpio_set_level(GPIO_NUM_1, 1);
    gpio_set_level(GPIO_NUM_2, 0);
    gpio_set_level(GPIO_NUM_3, 0);


    // dtrn_out -> led3
	esp_rom_gpio_connect_out_signal(3, U1DTR_OUT_IDX, 0, 0);


	// wlan active
	//esp_rom_gpio_connect_out_signal(2, GPIO_WLAN_ACTIVE_IDX, 0, 0);


    // init timers

    led1_timer = xTimerCreate("LED1 timer", 500 / portTICK_PERIOD_MS, pdTRUE, (void *) 2, led1_elapsed);
    //led2_timer = xTimerCreate("LED1 timer", 500 / portTICK_PERIOD_MS, pdTRUE, 3, led2_elapsed);
    //led3_timer = xTimerCreate("LED1 timer", 500 / portTICK_PERIOD_MS, pdTRUE, 4, led3_elapsed);

    // start timers
    xTimerStart(led1_timer, portMAX_DELAY);
    //xTimerStart(led2_timer, portMAX_DELAY);
    //xTimerStart(led3_timer, portMAX_DELAY);







    for (;;)
    {

    	// check for new state
    	xTaskNotifyWait(0, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);	// wait for event

    	if (ulNotifiedValue & LED_CMD_DISCONNECTED)
    	{
    		gpio_set_level(GPIO_NUM_2, 0);
    	}


    	if (ulNotifiedValue & LED_CMD_CONNECTED)
    	{
    		gpio_set_level(GPIO_NUM_2, 1);
    	}

    	if (ulNotifiedValue & LED_CMD_MODE_AP)
    	{
    		// fast blinking
    		xTimerChangePeriod(led1_timer, 250 / portTICK_PERIOD_MS, portMAX_DELAY);
    	}

    	if (ulNotifiedValue & LED_CMD_MODE_NORMAL)
		{
			// normal blinking
			xTimerChangePeriod(led1_timer, 500 / portTICK_PERIOD_MS, portMAX_DELAY);
		}


    	//switch (ledstate)
    	//{
    	//	case LED_STATE_INIT:
    	//		break;
    	//}

		// gpio_set_level(GPIO_NUM_1, 1);
		//vTaskDelay(500 / portTICK_PERIOD_MS);
		// gpio_set_level(GPIO_NUM_1, 0);
		//vTaskDelay(500 / portTICK_PERIOD_MS);
    }



}




static void button_task(void *pvParameters)
{
	static int btn_cnt = 0;
	static bool apmode = false;


	for (;;)
	{
		if (gpio_get_level(GPIO_NUM_0))
		{
			// button up
			btn_cnt = 0;
		}
		else
		{
			// button down
			if (btn_cnt == 30)	// ~3 seconds
			{
				// switch mode

				if (apmode)
				{
					// switch to normal mode
					xTaskNotify(h_led_task, LED_CMD_MODE_NORMAL, eSetBits);
					apmode = false;

					printf("Wifi disconnecting...\n");
					ESP_ERROR_CHECK(esp_wifi_stop() );


					ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
					ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &netCfg.wifi_config) );

					printf("Wifi connecting...\n");

					ESP_ERROR_CHECK(esp_wifi_start() );

					esp_wifi_connect();









				}
				else
				{
					// switch to ap mode
					xTaskNotify(h_led_task, LED_CMD_MODE_AP, eSetBits);
					apmode = true;

					printf("Wifi disconnecting...\n");
					ESP_ERROR_CHECK(esp_wifi_stop() );

					 wifi_config_t wifi_config_ap = {
							.ap = {
								.ssid = "Battery monitor AP",
								.password = "batt_mon",
								.authmode = WIFI_AUTH_WPA2_PSK,
								.max_connection = 4,
								.channel = 1
							}
						};

					ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP) );
					ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config_ap) );

					printf("Wifi AP initializing...\n");

					ESP_ERROR_CHECK(esp_wifi_start() );







				}

			}

			btn_cnt++;
		}

		// delay 100 ms
		vTaskDelay(100 / portTICK_PERIOD_MS);

	}
}



// node refresh timer callback
static void reboot_cb(TimerHandle_t xTimer)
{
	esp_restart();
}


void app_main(void)
{
	static httpd_handle_t server = NULL;

	// create reboot timer
	tmr_reboot = xTimerCreate("Reboot timer", 3000 / portTICK_PERIOD_MS, pdFALSE, (void *) 0, reboot_cb);

	// create led task
    xTaskCreate(led_task, "led_task", 2048, NULL, 16 , &h_led_task);



	//vTaskDelay(3000 / portTICK_PERIOD_MS);

	// initialize tinyUSB
	ESP_LOGI(TAG, "USB initialization");

	tinyusb_config_t tusb_cfg = { 0 }; // the configuration uses default values
	ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

	tinyusb_config_cdcacm_t amc_cfg = { 0 }; // the configuration uses default values
	ESP_ERROR_CHECK(tusb_cdc_acm_init(&amc_cfg));

	//esp_tusb_init_console(TINYUSB_CDC_ACM_0); // log to usb

	ESP_LOGI(TAG, "USB initialization DONE");


    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());


    esp_log_level_set(TAG, ESP_LOG_INFO);

    if (!busInit())
	{
		// fatal error, abort execution
		abort();
	}

//    for (int adrCnt=0; adrCnt < 10; adrCnt++)
//    {
//    	nodes[adrCnt].uid = 1234+adrCnt;
//    	nodes[adrCnt].address = adrCnt+1;
//    	nodes[adrCnt].flag_enabled = 1;
//    	strcpy(nodes[adrCnt].name, "TEST");
//    }

    // create pkt response queue
//    node_response_queue = xQueueCreate(1, sizeof(node_response_t*));
//
//    if (node_response_queue == NULL)
//    {
//    	// cannot create queue, abort execution
//    	abort();
//    }

   //comm_req_queue = xQueueCreate(10, sizeof(comm_req_t*));

    //if (comm_req_queue == NULL)
    //{
    //	// cannot create queue, abort execution
    //	abort();
    //}


    // init semaphores
    caldata.xSemaphore = xSemaphoreCreateBinary();

    if (caldata.xSemaphore == NULL)
    {
    	// cannot create semaphore, abort execution
    	abort();
    }


    setaddrData.xSemaphore = xSemaphoreCreateBinary();

    if (setaddrData.xSemaphore == NULL)
	{
		// cannot create semaphore, abort execution
		abort();
	}


    // init nvs

	esp_err_t ret = nvs_flash_init();

	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}

	ESP_ERROR_CHECK(ret);

	// load config from NVS
	netCfg.updInterval = 60;	// default value


	ESP_LOGI(TAG, "Trying to load group/node cfg from NVS...");

	nvs_handle_t hNvsCfg;

	ret = nvs_open("cfg", NVS_READONLY, &hNvsCfg);

	if (ret == ESP_OK)
	{
		// upd interval
		ret = nvs_get_u16(hNvsCfg, "upd_int", &netCfg.updInterval);

		if (ret != ESP_OK)
		{
			ESP_LOGW(TAG, "Unable to load update interval to NVS, using default!");
		}

		// groups
		size_t nvsGroupBufLen = sizeof(nvs_group_t) * MAX_GROUPS;
		nvs_group_t *nvsGroupBuf = malloc(nvsGroupBufLen);

		if (nvsGroupBuf == NULL)
			abort();

		ret = nvs_get_blob(hNvsCfg, "group_cfg", nvsGroupBuf, &nvsGroupBufLen);

		if (ret == ESP_OK)
		{
			// data loaded from NVS

			for (int i=0; i<MAX_GROUPS; i++)
			{
				groups[i].state = nvsGroupBuf[i].state;
				strncpy(groups[i].name, nvsGroupBuf[i].name, MAX_NAME);
				groups[i].name[MAX_NAME-1] = '\0';	// make sure last char is null for safety
			}
		}
		else
		{
			ESP_LOGW(TAG, "Unable to load group cfg from NVS, using defaults!");
		}

		free(nvsGroupBuf);

		// nodes
		size_t nvsNodeBufLen = sizeof(nvs_node_t) * MAX_NODES;
		nvs_node_t *nvsNodeBuf = malloc(nvsNodeBufLen);

		if (nvsNodeBuf == NULL)
			abort();

		ret = nvs_get_blob(hNvsCfg, "node_cfg", nvsNodeBuf, &nvsNodeBufLen);

		if (ret == ESP_OK)
		{
			// data loaded from NVS

			for (int i=0; i<MAX_NODES; i++)
			{
				nodes[i].address = nvsNodeBuf[i].address;
				nodes[i].group = nvsNodeBuf[i].group;
				nodes[i].group_pos = nvsNodeBuf[i].group_pos;
				nodes[i].flag_enabled = (nvsNodeBuf[i].state == NODE_STATE_ENABLED) ? 1 : 0;
				strncpy(nodes[i].name, nvsNodeBuf[i].name, MAX_NAME);
				nodes[i].name[MAX_NAME-1] = '\0';	// make sure last char is null for safety
			}
		}
		else
		{
			ESP_LOGW(TAG, "Unable to load node cfg from NVS, using defaults!");
		}

		free(nvsNodeBuf);

		nvs_close(hNvsCfg);

	}
	else
	{
		ESP_LOGW(TAG, "Unable to load group/node cfg from NVS, using defaults!");
	}



	// load network settings

	// set default config
	//netCfg.ip_addr = PP_HTONL(LWIP_MAKEU32(192,168,1,100));
	netCfg.ip_addr.addr = 0;	// use DHCP
	netCfg.ip_subnet.addr = 0;
	netCfg.ip_gw.addr = 0;
	netCfg.ip_dns.addr = 0;
	strcpy(netCfg.sntp_srv,"pool.ntp.org");
	strcpy(netCfg.timezone.tz,"UTC0");
	strcpy(netCfg.timezone.name,"Etc/UTC");

	netCfg.wifi_config.sta.ssid[0] = 0;
	netCfg.wifi_config.sta.password[0] = 0;
	netCfg.wifi_config.sta.pmf_cfg.capable = true;
	netCfg.wifi_config.sta.pmf_cfg.required = false;
	netCfg.wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;

	ESP_LOGI(TAG, "Trying to load net cfg from NVS...");

	ret = nvs_open("netcfg", NVS_READONLY, &hNvsCfg);

	if (ret == ESP_OK)
	{

		size_t strLen;

		// time zone
		strLen = sizeof(netCfg.timezone.tz);
		ret = nvs_get_str(hNvsCfg, "tz", netCfg.timezone.tz, &strLen);

		if (ret != ESP_OK)
		{
			ESP_LOGW(TAG, "Unable to load timezone cfg from NVS, using default (UTC)!");
		}

		// time zone name
		strLen = sizeof(netCfg.timezone.name);
		ret = nvs_get_str(hNvsCfg, "tzname", netCfg.timezone.name, &strLen);

		if (ret != ESP_OK)
		{
			strcpy(netCfg.timezone.tz,"UTC0");
			ESP_LOGW(TAG, "Unable to load timezone cfg from NVS, using default (UTC)!");
		}


		// SNTP server
		strLen = sizeof(netCfg.sntp_srv);
		ret = nvs_get_str(hNvsCfg, "sntp", netCfg.sntp_srv, &strLen);

		if (ret != ESP_OK)
		{
			ESP_LOGW(TAG, "Unable to load SNTP cfg from NVS, using default (pool.ntp.org)!");
		}

		// WiFi auth mode
		ret = nvs_get_u32(hNvsCfg, "wifi_auth", &netCfg.wifi_config.sta.threshold.authmode);

		if (ret != ESP_OK)
		{
			ESP_LOGW(TAG, "Unable to load WiFi auth cfg from NVS, using default!");
		}

		// WiFi SSID
		strLen = sizeof(netCfg.wifi_config.sta.ssid);
		ret = nvs_get_str(hNvsCfg, "wifi_ssid", (char*)netCfg.wifi_config.sta.ssid, &strLen);

		if (ret != ESP_OK)
		{
			netCfg.wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
			ESP_LOGW(TAG, "Unable to load WiFi SSID cfg from NVS, WiFi STA will be disabled!");
		}

		// WiFi PW
		strLen = sizeof(netCfg.wifi_config.sta.password);
		ret = nvs_get_str(hNvsCfg, "wifi_pw", (char*)netCfg.wifi_config.sta.password, &strLen);

		if (ret != ESP_OK)
		{
			netCfg.wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
			netCfg.wifi_config.sta.ssid[0] = 0;
			ESP_LOGW(TAG, "Unable to load WiFi PW cfg from NVS, WiFi STA will be disabled!");
		}

		// IP addr
		ret = nvs_get_u32(hNvsCfg, "ip_addr", &netCfg.ip_addr.addr);

		if (ret != ESP_OK)
		{
			ESP_LOGW(TAG, "Unable to load IP addr cfg from NVS, using default!");
		}

		// IP subnet
		ret = nvs_get_u32(hNvsCfg, "ip_subnet", &netCfg.ip_subnet.addr);

		if (ret != ESP_OK)
		{
			ESP_LOGW(TAG, "Unable to load IP subnet cfg from NVS, using default!");
		}

		// IP gw
		ret = nvs_get_u32(hNvsCfg, "ip_gw", &netCfg.ip_gw.addr);

		if (ret != ESP_OK)
		{
			ESP_LOGW(TAG, "Unable to load IP GW cfg from NVS, using default!");
		}


		// IP DNS
		ret = nvs_get_u32(hNvsCfg, "ip_dns", &netCfg.ip_dns.addr);

		if (ret != ESP_OK)
		{
			ESP_LOGW(TAG, "Unable to load IP DNS cfg from NVS, using default!");
		}


		nvs_close(hNvsCfg);
	}
	else
	{

		ESP_LOGW(TAG, "Unable to load net cfg from NVS, using defaults!");
	}




	printf("--- NETWORK CONFIG: ---\r\n");
	printf("WiFi SSID: %s\r\n", (char*)netCfg.wifi_config.sta.ssid);
	printf("WiFi PW: %s\r\n", (char*)netCfg.wifi_config.sta.password);
	printf("WiFi auth: %u\r\n", netCfg.wifi_config.sta.threshold.authmode);
	printf("IP address: %s\r\n", ip4addr_ntoa(&netCfg.ip_addr));
	printf("IP subnet: %s\r\n", ip4addr_ntoa(&netCfg.ip_subnet));
	printf("IP gateway: %s\r\n", ip4addr_ntoa(&netCfg.ip_gw));
	printf("IP DNS: %s\r\n", ip4addr_ntoa(&netCfg.ip_dns));
	printf("SNTP server: %s\r\n", netCfg.sntp_srv);
	printf("Timezone name: %s\r\n", netCfg.timezone.name);
	printf("Timezone data: %s\r\n", netCfg.timezone.tz);
	printf("--- END NETWORK CONFIG ---\r\n");



    // init network
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *if_sta = esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();


    // ip settings

    if (netCfg.ip_addr.addr != 0)
    {
    	// static ip
        esp_netif_dhcpc_stop(if_sta);
        esp_netif_ip_info_t info;

        info.ip.addr = netCfg.ip_addr.addr;
        info.gw.addr = netCfg.ip_gw.addr;
        info.netmask.addr = netCfg.ip_subnet.addr;
        esp_netif_set_ip_info(if_sta, &info);


        esp_netif_dns_info_t dns;
        dns.ip.type = ESP_IPADDR_TYPE_V4;
        dns.ip.u_addr.ip4.addr = netCfg.ip_dns.addr;

        esp_netif_set_dns_info(if_sta, ESP_NETIF_DNS_MAIN, &dns);

    }






    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STOP, &disconnect_handler, &server));




    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &netCfg.wifi_config) );









    printf("Wifi connecting...\n");

    ESP_ERROR_CHECK(esp_wifi_start() );

    esp_wifi_connect();








	/* Mark current app as valid */
	const esp_partition_t *partition = esp_ota_get_running_partition();
	printf("Currently running partition: %s\r\n", partition->label);

	esp_ota_img_states_t ota_state;
	if (esp_ota_get_state_partition(partition, &ota_state) == ESP_OK) {
		if (ota_state == ESP_OTA_IMG_PENDING_VERIFY) {
			esp_ota_mark_app_valid_cancel_rollback();
		}
	}






	ESP_LOGI(TAG, "Starting webserver");
	server = start_webserver();



	// update time from NTP

	time_t now;
	struct tm timeinfo;


	char strftime_buf[64];

	setenv("TZ", netCfg.timezone.tz, 1);
	tzset();

	time(&now);
	localtime_r(&now, &timeinfo);
	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
	ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);


	ESP_LOGI(TAG, "Initializing SNTP");
	sntp_setoperatingmode(SNTP_OPMODE_POLL);
	sntp_setservername(0, netCfg.sntp_srv);
	sntp_init();






    //Create a task to handler UART event from ISR
   // xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);


    xTaskCreate(comm_task, "comm_task", 2048, NULL, 5, &h_comm_task);










    tmr_node_refresh = xTimerCreate("Node refresh timer", ((uint32_t)netCfg.updInterval * 1000) / portTICK_PERIOD_MS, pdTRUE, (void *) 1, node_refresh_trig);

    // start timer
    xTimerStart(tmr_node_refresh, portMAX_DELAY);

    // initial data collection
    xTaskNotify(h_comm_task, COMM_REQ_MEASURE, eSetBits);


    // create button task
    xTaskCreate(button_task, "button_task", 2048, NULL, 16 , &h_button_task);


    //comm_req_t* commReq;


    for (;;)
	{
    	time(&now);
    	localtime_r(&now, &timeinfo);
    	strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    	ESP_LOGI(TAG, "The current date/time is: %s", strftime_buf);


       // gpio_set_level(GPIO_NUM_1, 1);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
       // gpio_set_level(GPIO_NUM_1, 0);
       // vTaskDelay(500 / portTICK_PERIOD_MS);


        // allocate memory for request
//    	commReq = (comm_req_t*)malloc(sizeof(comm_req_t));
//
//    	if (commReq != NULL)
//    	{
//			commReq->addr = 1;
//			commReq->reqType = COMM_REQ_MEAS;
//			commReq->dat_len = 0;
//			commReq->dat = NULL;
//
//			printf("Meas request\n");
//
//			if (xQueueSend(comm_req_queue, &commReq, 0) != pdTRUE)
//			{
//				printf("Meas request failed\n");
//				// failed, free memory used by request
//				free(commReq);
//			}
//    	}






		// send measurement start to queue



//        // send test packet
//        txBuf[1] = 8;			// pkt len
//        txBuf[2] = 1;			// dst addr
//        txBuf[3] = seq++;   // seq
//        txBuf[4] = (0x02 << 2);	// cmd
//
//
//        cobs_encode(txBuf[1]+1, txBuf);
//
//
//        printf("[UART TX]: %d\r\n", txBuf[1]+1);
//
//		for (int i=0; i<txBuf[1]+1; i++)
//		{
//			printf("%02X",txBuf[i]);
//		}
//
//		printf("\r\n");
//
//		uint8_t zeroByte[] = {0};
//
//		uart_write_bytes(EX_UART_NUM, zeroByte, 1);
//        uart_write_bytes(EX_UART_NUM, txBuf, txBuf[1]+1);
//        uart_write_bytes(EX_UART_NUM, zeroByte, 1);

	}

}
