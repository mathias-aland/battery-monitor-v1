/*
 * websrv.c
 *
 *  Created on: 7 dec. 2021
 *      Author: Mathias
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include <esp_http_server.h>
#include <esp_ota_ops.h>
#include "esp_rom_crc.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include <nvs_flash.h>
#include <cJSON.h>
#include <sys/param.h>
#include "common.h"
#include "cobs.h"
#include "bus.h"


// set addr HTML
extern const uint8_t setaddr_html_start[] asm("_binary_setaddr_html_start");
extern const uint8_t setaddr_html_end[] asm("_binary_setaddr_html_end");

// node edit html
extern const uint8_t editnodes_html_start[] asm("_binary_editnodes_html_start");
extern const uint8_t editnodes_html_end[] asm("_binary_editnodes_html_end");

// nodes html
extern const uint8_t nodes_html_start[] asm("_binary_nodes_html_start");
extern const uint8_t nodes_html_end[] asm("_binary_nodes_html_end");


// node FW portal
extern const uint8_t nodefw_html_start[] asm("_binary_nodefw_html_start");
extern const uint8_t nodefw_html_end[] asm("_binary_nodefw_html_end");

// OTA update portal
extern const uint8_t ota_html_start[] asm("_binary_ota_html_start");
extern const uint8_t ota_html_end[] asm("_binary_ota_html_end");

// network settings html
extern const uint8_t network_html_start[] asm("_binary_network_html_start");
extern const uint8_t network_html_end[] asm("_binary_network_html_end");

// cal settings html
extern const uint8_t cal_html_start[] asm("_binary_cal_html_start");
extern const uint8_t cal_html_end[] asm("_binary_cal_html_end");




static const char *TAG = "websrv";

// Static content HTTP GET
esp_err_t static_get_handler(httpd_req_t *req)
{
	printf("HTTPD URI request: ");
	printf(req->uri);
	printf("\r\n");


	if (strcmp(req->uri, "/ui/ota") == 0)
	{
		httpd_resp_send(req, (const char *) ota_html_start, ota_html_end - ota_html_start);
		return ESP_OK;
	}
	else if (strcmp(req->uri, "/ui/nodefw") == 0)
	{
		httpd_resp_send(req, (const char *) nodefw_html_start, nodefw_html_end - nodefw_html_start);
		return ESP_OK;
	}
	else if (strcmp(req->uri, "/") == 0)
	{
		httpd_resp_send(req, (const char *) nodes_html_start, nodes_html_end - nodes_html_start);
		return ESP_OK;
	}
	else if (strcmp(req->uri, "/ui/editnodes") == 0)
	{
		httpd_resp_send(req, (const char *) editnodes_html_start, editnodes_html_end - editnodes_html_start);
		return ESP_OK;
	}
	else if (strcmp(req->uri, "/ui/setaddr") == 0)
	{
		httpd_resp_send(req, (const char *) setaddr_html_start, setaddr_html_end - setaddr_html_start);
		return ESP_OK;
	}
	else if (strcmp(req->uri, "/ui/network") == 0)
	{
		httpd_resp_send(req, (const char *) network_html_start, network_html_end - network_html_start);
		return ESP_OK;
	}
	else if (strcmp(req->uri, "/ui/cal") == 0)
	{
		httpd_resp_send(req, (const char *) cal_html_start, cal_html_end - cal_html_start);
		return ESP_OK;
	}
	else
	{
		httpd_resp_send_404(req);
		return ESP_OK;
	}




}



esp_err_t cfg_get_handler(httpd_req_t *req)
{
	const esp_partition_t *nvsPart = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, "nvs");

	if (nvsPart == NULL)
	{
		// error
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "NVS partition not found!");
		return ESP_FAIL;
	}

	ESP_LOGI(TAG, "Found nvs partition at %#010x, size %#010x", nvsPart->address, nvsPart->size);

	// mmap nvs partition
	const void *map_ptr;
	spi_flash_mmap_handle_t map_handle;
	esp_err_t ret = esp_partition_mmap(nvsPart, 0, nvsPart->size, SPI_FLASH_MMAP_DATA, &map_ptr, &map_handle);

	if (ret != ESP_OK)
	{
		// error
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_partition_mmap failed!");
		return ESP_FAIL;
	}

	// send data
	ESP_LOGI(TAG, "Sending data to client");
	httpd_resp_set_type(req, "application/octet-stream");
	httpd_resp_send(req, map_ptr, nvsPart->size);

	spi_flash_munmap(map_handle);

	return ESP_OK;

}


// software update routines

esp_err_t nodefwupdate_post_handler(httpd_req_t *req)
{
	char buf[1000];
	int remaining = req->content_len;

	// try to take mutex
	if( xSemaphoreTake( xNodeFwMutex, 100 / portTICK_PERIOD_MS ) == pdFALSE )	// 100 ms
	{
		// cannot take mutex
		// update already in progress
		httpd_resp_sendstr(req, "Update already in progress, cannot continue!");
		return ESP_OK;
	}

	nodefw.status = FWUPD_STATUS_NO_DATA;
	memset(nodefw.fwdata,0xFF,NODE_FW_SIZE);

	int curPos = 0;

	while (remaining > 0) {
		int recv_len = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));

		// Timeout Error: Just retry
		if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
			continue;

		// Serious Error: Abort OTA
		} else if (recv_len <= 0) {
			xSemaphoreGive(xNodeFwMutex);
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
			return ESP_FAIL;
		}

		// Successful Upload: Flash firmware chunk
		if (curPos + recv_len > NODE_FW_SIZE)
		{
			// error, too large file
			xSemaphoreGive(xNodeFwMutex);
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Too much data sent");
			return ESP_FAIL;
		}

		memcpy(&nodefw.fwdata[curPos],buf,recv_len);
		curPos += recv_len;
		remaining -= recv_len;
	}

	// CRC check
	nodefw.crc = nodefw.fwdata[NODE_FW_SIZE-2] << 8 | nodefw.fwdata[NODE_FW_SIZE-1];
	nodefw.crc_calc = ~esp_rom_crc16_be((uint16_t)~0xffff, nodefw.fwdata, NODE_FW_SIZE-2);

	if (nodefw.crc != nodefw.crc_calc)
		nodefw.status = FWUPD_STATUS_CRC_ERR;
	else
		nodefw.status = FWUPD_STATUS_IDLE;


	xSemaphoreGive(xNodeFwMutex);

	char str[200];

	sprintf(str,"Upload complete, expected CRC: %04X, calculated CRC: %04X",nodefw.crc,nodefw.crc_calc);

	httpd_resp_sendstr(req, str);

	return ESP_OK;
}


//esp_err_t nodefw_do_upd_post_handler(httpd_req_t *req)
//{
//	char buf[16];
//
//
//	/* Truncate if content length larger than the buffer */
//	size_t recv_size = MIN(req->content_len, sizeof(buf));
//
//	int ret = httpd_req_recv(req, buf, recv_size);
//
//	if (ret <= 0) {  /* 0 return value indicates connection closed */
//		/* Check if timeout occurred */
//		if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
//			/* In case of timeout one can choose to retry calling
//			 * httpd_req_recv(), but to keep it simple, here we
//			 * respond with an HTTP 408 (Request Timeout) error */
//			httpd_resp_send_408(req);
//		}
//		/* In case of error, returning ESP_FAIL will
//		 * ensure that the underlying socket is closed */
//		return ESP_FAIL;
//	}
//
//
//	if (strncmp(buf,"all",recv_size) == 0)
//	{
//		// start update on all nodes
//
//		if ((nodefw.status == FWUPD_STATUS_DONE) || (nodefw.status == FWUPD_STATUS_IDLE))
//		{
//			// start update
//			ESP_LOGI(TAG, "Starting FW update on all nodes!");
//			nodefw.upd_type = FWUPD_TYPE_ALL;
//			xTaskNotify(h_comm_task, COMM_REQ_FWUPD, eSetBits);	// trigger fw update
//		}
//		else
//		{
//			httpd_resp_sendstr(req, "ERR");
//			return ESP_OK;
//		}
//	}
//	else if (strncmp(buf,"safe",recv_size) == 0)
//	{
//		if ((nodefw.status == FWUPD_STATUS_DONE) || (nodefw.status == FWUPD_STATUS_IDLE))
//		{
//			// start update
//			ESP_LOGI(TAG, "Starting FW update on node in failsafe mode!");
//			nodefw.upd_type = FWUPD_TYPE_FAILSAFE;
//			xTaskNotify(h_comm_task, COMM_REQ_FWUPD, eSetBits);	// trigger fw update
//		}
//		else
//		{
//			httpd_resp_sendstr(req, "ERR");
//			return ESP_OK;
//		}
//
//	}
//	else
//	{
//		httpd_resp_sendstr(req, "ERR");
//		return ESP_OK;
//	}
//
//	httpd_resp_sendstr(req, "OK");
//	return ESP_OK;
//
//}


void print_api_help()
{

}


esp_err_t api_get_handler(httpd_req_t *req)
{
	char* buf;
	size_t buf_len;


	buf_len = httpd_req_get_url_query_len(req) + 1;

	if (buf_len > 1) {
		buf = (char*)malloc(buf_len);
		if(!buf){
			httpd_resp_send_500(req);
			return ESP_FAIL;
		}


		if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK)
		{
			ESP_LOGI(TAG, "Found URL query => %s", buf);

			char param[32];

			if (httpd_query_key_value(buf, "op", param, sizeof(param)) == ESP_OK)
			{
				ESP_LOGI(TAG, "Found URL query parameter => op=%s", param);


			}
			else
			{
				print_api_help();
			}

			httpd_resp_sendstr(req, buf);

		}
		else
		{
			httpd_resp_sendstr(req, "Error getting query string!");
		}




		free(buf);


	}
	else
	{
		print_api_help();



		cJSON *root;
		root = cJSON_CreateObject();


		// wifi stats
		wifi_ap_record_t ap;
		if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK)
		{
			cJSON_AddNumberToObject(root, "wifi_rssi", ap.rssi);
		}
		else
		{
			cJSON_AddNullToObject(root, "wifi_rssi");
		}

		// free heap
		cJSON_AddNumberToObject(root, "heap_int", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
		cJSON_AddNumberToObject(root, "heap_ext", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

		// enabled groups
		cJSON *jGroups = cJSON_CreateArray();

		for (int i=0; i<MAX_GROUPS; i++)
		{
			if (groups[i].state == GROUP_STATE_ENABLED)
			{
				cJSON *jGroup = cJSON_CreateObject();
				cJSON_AddItemToObject(jGroup, "id", cJSON_CreateNumber(i+1));

				// group name
				cJSON_AddStringToObject(jGroup, "name", groups[i].name);

				// add to array
				cJSON_AddItemToArray(jGroups, jGroup);

			}
		}

		cJSON_AddItemToObject(root, "groups", jGroups);

		cJSON *jNodes = cJSON_CreateArray();

		for (int i=0; i<MAX_NODES; i++)
		{
			if ((nodes[i].address != 0) && (nodes[i].flag_enabled))
			{
				cJSON *jNode = cJSON_CreateObject();

				// node address
				cJSON_AddNumberToObject(jNode, "addr", nodes[i].address);

				// node name
				cJSON_AddStringToObject(jNode, "name", nodes[i].name);

				// node group
				cJSON_AddNumberToObject(jNode, "grp", nodes[i].group);

				// position in group
				cJSON_AddNumberToObject(jNode, "pos", nodes[i].group_pos);

				// status
				if (nodes[i].flag_error)
					cJSON_AddStringToObject(jNode, "status", "Error");
				else
					cJSON_AddStringToObject(jNode, "status", "OK");

				// total voltage
				cJSON_AddNumberToObject(jNode, "vbat", nodes[i].meas_data[5]);


				// vdd
				cJSON_AddNumberToObject(jNode, "vdd", nodes[i].meas_data[6]);

				// temperature
				cJSON_AddNumberToObject(jNode, "temp", nodes[i].meas_data[7]);

				cJSON *jMeasures = cJSON_CreateArray();

				// Cell 1 voltage
				cJSON_AddItemToArray(jMeasures,cJSON_CreateNumber( nodes[i].meas_data[0]));

				// Cell 2 voltage
				cJSON_AddItemToArray(jMeasures,cJSON_CreateNumber( nodes[i].meas_data[1] - nodes[i].meas_data[0]));

				// Cell 3 voltage
				cJSON_AddItemToArray(jMeasures,cJSON_CreateNumber( nodes[i].meas_data[2] - nodes[i].meas_data[1]));

				// Cell 4 voltage
				cJSON_AddItemToArray(jMeasures,cJSON_CreateNumber( nodes[i].meas_data[3] - nodes[i].meas_data[2]));

				// Cell 5 voltage
				cJSON_AddItemToArray(jMeasures,cJSON_CreateNumber( nodes[i].meas_data[4] - nodes[i].meas_data[3]));

				// Cell 6 voltage
				cJSON_AddItemToArray(jMeasures,cJSON_CreateNumber( nodes[i].meas_data[5] - nodes[i].meas_data[4]));

				cJSON_AddItemToObject(jNode, "cells", jMeasures);
				cJSON_AddItemToArray(jNodes, jNode);

			}
		}

		cJSON_AddItemToObject(root, "nodes", jNodes);

		char *json = NULL;
		json = cJSON_Print(root);
		httpd_resp_sendstr(req, json);
		free(json);
		cJSON_Delete(root);





	}




	return ESP_OK;
}


esp_err_t api_post_handler(httpd_req_t *req)
{
	char *buf;

	buf = malloc(2048);	// allocate memory for buffer

	if (buf == NULL)
	{
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
		return ESP_FAIL;
	}



	/* Truncate if content length larger than the buffer */
	size_t recv_size = MIN(req->content_len, 2048);

	int ret = httpd_req_recv(req, buf, recv_size);

	if (ret <= 0) {  /* 0 return value indicates connection closed */
		/* Check if timeout occurred */
		if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
			/* In case of timeout one can choose to retry calling
			 * httpd_req_recv(), but to keep it simple, here we
			 * respond with an HTTP 408 (Request Timeout) error */
			httpd_resp_send_408(req);
		}
		/* In case of error, returning ESP_FAIL will
		 * ensure that the underlying socket is closed */
		free(buf);
		return ESP_FAIL;
	}



	printf("recv: %.*s\n", recv_size, buf);



	cJSON *root = cJSON_ParseWithLength(buf, recv_size);

	if (root == NULL)
	{
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON parse error");
		free(buf);
		return ESP_FAIL;
	}


	cJSON *op = cJSON_GetObjectItem(root,"cmd");

	if (!cJSON_IsString(op))
	{
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON cmd is not a string");
		cJSON_Delete(root);
		free(buf);
		return ESP_FAIL;
	}


	// init response JSON
	cJSON *rootResp;
	rootResp = cJSON_CreateObject();

	if (rootResp == NULL)
	{
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
		cJSON_Delete(root);
		free(buf);
		return ESP_FAIL;
	}


	// add cmd to reply
	if (cJSON_AddStringToObject(rootResp, "cmd", op->valuestring) == NULL)
	{
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
		goto api_post_handler_fail;
	}




	if (strcmp(op->valuestring,"getmeas") == 0)
	{
		// get measurements from node
		cJSON *addr = cJSON_GetObjectItem(root,"address");

		if (!cJSON_IsNumber(addr))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON address is not a number");
			goto api_post_handler_fail;
		}


		if ((addr->valueint < 1) || (addr->valueint > 255))
		{
			// invalid address
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON address is invalid");
			goto api_post_handler_fail;
		}

		int16_t meas_data[8];

		if (!getMeas(addr->valueint, meas_data))
		{
			// Error
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to get data!");
			goto api_post_handler_fail;
		}

		// set address in reply
		cJSON_AddNumberToObject(rootResp, "address", addr->valueint);

		// set status in reply
		cJSON_AddTrueToObject(rootResp, "status");

		// cal data arrays
		cJSON *measDataJSON = cJSON_AddArrayToObject(rootResp, "meas");
		if (measDataJSON == NULL)
		{
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
			goto api_post_handler_fail;
		}

		for (int ch=0; ch<8; ch++)
		{
			cJSON_AddItemToArray(measDataJSON,cJSON_CreateNumber(meas_data[ch]));
		}



	}
	else if (strcmp(op->valuestring,"setaddr") == 0)
	{
		// set addr

		cJSON *addr = cJSON_GetObjectItem(root,"address");

		if (!cJSON_IsNumber(addr))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Address is not a number");
			goto api_post_handler_fail;
		}

		if ((addr->valueint < 1) || (addr->valueint > 255))
		{
			// invalid address
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Address is out of range!");
			goto api_post_handler_fail;
		}

		// set address
		if (!setAddr((uint8_t)addr->valueint))
		{
			// Error
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set address!");
			goto api_post_handler_fail;
		}

		// set address in reply
		cJSON_AddNumberToObject(rootResp, "address", addr->valueint);

		// set status in reply
		cJSON_AddTrueToObject(rootResp, "status");

	}
	else if (strcmp(op->valuestring,"getcal") == 0)
	{
		// get cal data for node
		cJSON *addr = cJSON_GetObjectItem(root,"address");

		if (!cJSON_IsNumber(addr))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON address is not a number");
			goto api_post_handler_fail;
		}


		if ((addr->valueint < 1) || (addr->valueint > 255))
		{
			// invalid address
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON address is invalid");
			goto api_post_handler_fail;
		}

		cal_data_t calData;

		calData.address = addr->valueint;

		if (!getCalData(&calData))
		{
			// Error
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to get cal data!");
			goto api_post_handler_fail;
		}

		// set address in reply
		cJSON_AddNumberToObject(rootResp, "address", addr->valueint);

		// set status in reply
		cJSON_AddTrueToObject(rootResp, "status");

		// cal data arrays
		cJSON *calDataJSON = cJSON_AddArrayToObject(rootResp, "offset");
		if (calDataJSON == NULL)
		{
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
			goto api_post_handler_fail;
		}

		for (int ch=0; ch<8; ch++)
		{
			cJSON_AddItemToArray(calDataJSON,cJSON_CreateNumber(calData.cal_offset[ch]));
		}

		calDataJSON = cJSON_AddArrayToObject(rootResp, "fullscale");
		if (calDataJSON == NULL)
		{
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
			goto api_post_handler_fail;
		}

		for (int ch=0; ch<8; ch++)
		{
			cJSON_AddItemToArray(calDataJSON,cJSON_CreateNumber(calData.cal_fullscale[ch]));
		}


	}
	else if (strcmp(op->valuestring,"nodefwupd") == 0)
	{
		// start FW update on node
		cJSON *addr = cJSON_GetObjectItem(root,"address");

		if (!cJSON_IsNumber(addr))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON address is not a number");
			goto api_post_handler_fail;
		}

		if ((addr->valueint < 0) || (addr->valueint > 254))
		{
			// invalid address
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON address is invalid");
			goto api_post_handler_fail;
		}


		if ((nodefw.status == FWUPD_STATUS_DONE) || (nodefw.status == FWUPD_STATUS_IDLE) || (nodefw.status == FWUPD_STATUS_FAIL))
		{
			// start update
			nodefw.addr = addr->valueint;
			nodefw.status = FWUPD_STATUS_PEND;
			ESP_LOGI(TAG, "Starting node FW update!");
			xTaskNotify(h_comm_task, COMM_REQ_FWUPD, eSetBits);	// trigger fw update
		}
		else
		{
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid FW status!");
			goto api_post_handler_fail;
		}

		// set status in reply
		cJSON_AddTrueToObject(rootResp, "status");


	}
	else if (strcmp(op->valuestring,"eraseconfig") == 0)
	{
		// erase NVS

		esp_err_t ret = nvs_flash_erase();

		if (ret != ESP_OK)
		{
			// error
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to erase NVS!");
			goto api_post_handler_fail;
		}

		if( xTimerStart( tmr_reboot, 5000 / portTICK_PERIOD_MS ) == pdFAIL)
		{
			// cannot start timer
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Busy, try again later!");
			goto api_post_handler_fail;
		}
		else
		{
			cJSON_AddTrueToObject(rootResp, "status");
		}
	}
	else if (strcmp(op->valuestring,"reboot") == 0)
	{
		if( xTimerStart( tmr_reboot, 5000 / portTICK_PERIOD_MS ) == pdFAIL)
		{
			// cannot start timer
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Busy, try again later!");
			goto api_post_handler_fail;
		}
		else
		{
			cJSON_AddTrueToObject(rootResp, "status");
		}
	}
	else if (strcmp(op->valuestring,"nodefwstat") == 0)
	{
		// get FW update status
		cJSON_AddNumberToObject(rootResp, "address", nodefw.addr);
		cJSON_AddNumberToObject(rootResp, "status", nodefw.status);
		cJSON_AddNumberToObject(rootResp, "crc", nodefw.crc);
		cJSON_AddNumberToObject(rootResp, "crccalc", nodefw.crc_calc);
		cJSON_AddNumberToObject(rootResp, "stage", nodefw.curr_stage);
		cJSON_AddNumberToObject(rootResp, "block", nodefw.curr_block);
		cJSON_AddNumberToObject(rootResp, "total", FW_BLOCKS-1);
	}
	else if (strcmp(op->valuestring,"savecal") == 0)
	{
		// save cal data for node
		cJSON *addr = cJSON_GetObjectItem(root,"address");

		if (!cJSON_IsNumber(addr))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON address is not a number");
			goto api_post_handler_fail;
		}


		if ((addr->valueint < 1) || (addr->valueint > 255))
		{
			// invalid address
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON address is invalid");
			goto api_post_handler_fail;
		}

		if (!saveCal(addr->valueint))
		{
			// Error
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save cal data!");
			goto api_post_handler_fail;
		}

		// set status in reply
		cJSON_AddTrueToObject(rootResp, "status");


	}

	else if (strcmp(op->valuestring,"setcal") == 0)
	{
		// set cal data for node
		cJSON *addr = cJSON_GetObjectItem(root,"address");

		if (!cJSON_IsNumber(addr))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON address is not a number");
			goto api_post_handler_fail;
		}


		if ((addr->valueint < 1) || (addr->valueint > 255))
		{
			// invalid address
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON address is invalid");
			goto api_post_handler_fail;
		}

		cal_data_t calData;

		calData.address = addr->valueint;



		// set address in reply
		cJSON_AddNumberToObject(rootResp, "address", addr->valueint);





		int i;

		cJSON *calDataArray;
		cJSON *calDataItem;

		// cal data offset
		calDataArray = cJSON_GetObjectItem(root,"offset");

		if (!cJSON_IsArray(calDataArray))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Cal data is not an array");
			goto api_post_handler_fail;
		}


		// count number of items in array
		if (cJSON_GetArraySize(calDataArray) != 8)
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Cal data array incorrect length");
			goto api_post_handler_fail;
		}




		i = 0;

		cJSON_ArrayForEach(calDataItem, calDataArray)
		{

			if (!cJSON_IsNumber(calDataItem))
			{
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not a number!");
				goto api_post_handler_fail;
			}

			if ((calDataItem->valueint < -32768) || (calDataItem->valueint > 32767))
			{
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Out of range!");
				goto api_post_handler_fail;
			}

			calData.cal_offset[i] = (int16_t)calDataItem->valueint;

			i++;


		}


		// cal data fullscale
		calDataArray = cJSON_GetObjectItem(root,"fullscale");

		if (!cJSON_IsArray(calDataArray))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Cal data is not an array");
			goto api_post_handler_fail;
		}


		// count number of items in array
		if (cJSON_GetArraySize(calDataArray) != 8)
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Cal data array incorrect length");
			goto api_post_handler_fail;
		}

		i = 0;

		cJSON_ArrayForEach(calDataItem, calDataArray)
		{

			if (!cJSON_IsNumber(calDataItem))
			{
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Not a number!");
				goto api_post_handler_fail;
			}

			if ((calDataItem->valueint < 0) || (calDataItem->valueint > 65535))
			{
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Out of range!");
				goto api_post_handler_fail;
			}

			calData.cal_fullscale[i] = (uint16_t)calDataItem->valueint;

			i++;
		}



		if (!setCalData(&calData))
		{
			// Error
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to set cal data!");
			goto api_post_handler_fail;
		}

		// set status in reply
		cJSON_AddTrueToObject(rootResp, "status");


	}
	else if (strcmp(op->valuestring,"getnodecfg") == 0)
	{
		// get group/node config
		// no parameters needed

		// update interval
		if (cJSON_AddNumberToObject(rootResp, "interval", netCfg.updInterval) == NULL)
		{
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
			goto api_post_handler_fail;
		}

		// populate groups data in reply

		// add array to root
		cJSON *groupsJSON = cJSON_AddArrayToObject(rootResp, "groups");
		if (groupsJSON == NULL)
		{
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
			goto api_post_handler_fail;
		}

		for (int i=0; i<MAX_GROUPS; i++)
		{
			// create group JSON object
			cJSON *groupJSON = cJSON_CreateObject();
			if (groupJSON == NULL)
			{
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
				goto api_post_handler_fail;
			}

			// add object to array
			cJSON_AddItemToArray(groupsJSON,groupJSON);


			// group name
			if (cJSON_AddStringToObject(groupJSON, "name", groups[i].name) == NULL)
			{
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
				goto api_post_handler_fail;
			}

			// group enabled state
			if (cJSON_AddBoolToObject(groupJSON, "en", groups[i].state == GROUP_STATE_ENABLED) == NULL)
			{
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
				goto api_post_handler_fail;
			}

		}



		// populate nodes data in reply

		// add array to root
		cJSON *nodesJSON = cJSON_AddArrayToObject(rootResp, "nodes");
		if (nodesJSON == NULL)
		{
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
			goto api_post_handler_fail;
		}

		for (int i=0; i<MAX_NODES; i++)
		{
			// create node JSON object
			cJSON *nodeJSON = cJSON_CreateObject();
			if (nodeJSON == NULL)
			{
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
				goto api_post_handler_fail;
			}

			// add object to array
			cJSON_AddItemToArray(nodesJSON,nodeJSON);


			// node address
			if (cJSON_AddNumberToObject(nodeJSON, "addr", nodes[i].address) == NULL)
			{
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
				goto api_post_handler_fail;
			}

			// node group
			if (cJSON_AddNumberToObject(nodeJSON, "grp", nodes[i].group) == NULL)
			{
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
				goto api_post_handler_fail;
			}

			// node pos
			if (cJSON_AddNumberToObject(nodeJSON, "pos", nodes[i].group_pos) == NULL)
			{
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
				goto api_post_handler_fail;
			}

			// node name
			if (cJSON_AddStringToObject(nodeJSON, "name", nodes[i].name) == NULL)
			{
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
				goto api_post_handler_fail;
			}

			// node enabled state
			if (cJSON_AddBoolToObject(nodeJSON, "en", nodes[i].flag_enabled) == NULL)
			{
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
				goto api_post_handler_fail;
			}



		}






	}
	else if (strcmp(op->valuestring,"setnodecfg") == 0)
	{
		// set group/node config

		// TODO: should do some checking and probably pause/reset data collection during update

		int i;

		// set new update interval
		cJSON *jUpdInt = cJSON_GetObjectItem(root,"interval");


		if (!cJSON_IsNumber(jUpdInt))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON update interval is not a number");
			goto api_post_handler_fail;
		}


		if ((jUpdInt->valueint < 10) || (jUpdInt->valueint > 65535))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON update interval is invalid");
			goto api_post_handler_fail;
		}


		netCfg.updInterval = jUpdInt->valueint;

		// stop data collection

		// try to take mutex
		if( xSemaphoreTake( xNodeCfgMutex, 5000 / portTICK_PERIOD_MS ) == pdFALSE )	// 5 seconds should be enough
		{
			// cannot take mutex
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Busy, try again later!");
			goto api_post_handler_fail;
		}

		// update groups
		cJSON *grps = cJSON_GetObjectItem(root,"groups");

		if (!cJSON_IsArray(grps))
		{
			xSemaphoreGive(xNodeCfgMutex);
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON groups is not an array");
			goto api_post_handler_fail;
		}


		// count number of items in array
		if (cJSON_GetArraySize(grps) != MAX_GROUPS)
		{
			xSemaphoreGive(xNodeCfgMutex);
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON groups array incorrect length");
			goto api_post_handler_fail;
		}


		cJSON *grp;

		i = 0;

		cJSON_ArrayForEach(grp, grps)
		{
			cJSON *grp_name = cJSON_GetObjectItem(grp,"name");
			cJSON *grp_en = cJSON_GetObjectItem(grp,"en");

			if (!cJSON_IsString(grp_name))
			{
				xSemaphoreGive(xNodeCfgMutex);
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON group name is not a string");
				goto api_post_handler_fail;
			}


			if (!cJSON_IsBool(grp_en))
			{
				xSemaphoreGive(xNodeCfgMutex);
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON group enabled state is not a bool");
				goto api_post_handler_fail;
			}

			// update entry
			strncpy(groups[i].name, grp_name->valuestring, MAX_NAME);
			groups[i].name[MAX_NAME-1] = '\0';	// string will not be null terminated if too long, add manually for safety

			if (cJSON_IsTrue(grp_en))
				groups[i].state = GROUP_STATE_ENABLED;
			else
				groups[i].state = GROUP_STATE_DISABLED;

			i++;


		}


		// update nodes
		cJSON *json_nodes = cJSON_GetObjectItem(root,"nodes");

		if (!cJSON_IsArray(json_nodes))
		{
			xSemaphoreGive(xNodeCfgMutex);
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON nodes is not an array");
			goto api_post_handler_fail;
		}

		// count number of items in array
		if (cJSON_GetArraySize(json_nodes) != MAX_NODES)
		{
			xSemaphoreGive(xNodeCfgMutex);
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON nodes array incorrect length");
			goto api_post_handler_fail;
		}

		cJSON *json_node;

		i = 0;

		cJSON_ArrayForEach(json_node, json_nodes)
		{

			cJSON *json_node_addr = cJSON_GetObjectItem(json_node,"addr");
			cJSON *json_node_grp = cJSON_GetObjectItem(json_node,"grp");
			cJSON *json_node_pos = cJSON_GetObjectItem(json_node,"pos");
			cJSON *json_node_name = cJSON_GetObjectItem(json_node,"name");
			cJSON *json_node_en = cJSON_GetObjectItem(json_node,"en");

			// address
			if (!cJSON_IsNumber(json_node_addr))
			{
				xSemaphoreGive(xNodeCfgMutex);
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON node addr is not a number");
				goto api_post_handler_fail;
			}

			if ((json_node_addr->valueint < 0) || (json_node_addr->valueint > 254))
			{
				xSemaphoreGive(xNodeCfgMutex);
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON node addr out of range");
				goto api_post_handler_fail;
			}

			// group
			if (!cJSON_IsNumber(json_node_grp))
			{
				xSemaphoreGive(xNodeCfgMutex);
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON node grp is not a number");
				goto api_post_handler_fail;
			}

			if ((json_node_grp->valueint < 0) || (json_node_grp->valueint > 8))
			{
				xSemaphoreGive(xNodeCfgMutex);
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON node grp out of range");
				goto api_post_handler_fail;
			}


			// group pos
			if (!cJSON_IsNumber(json_node_pos))
			{
				xSemaphoreGive(xNodeCfgMutex);
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON node pos is not a number");
				goto api_post_handler_fail;
			}

			if ((json_node_pos->valueint < 0) || (json_node_pos->valueint > 4))
			{
				xSemaphoreGive(xNodeCfgMutex);
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON node pos out of range");
				goto api_post_handler_fail;
			}


			// name
			if (!cJSON_IsString(json_node_name))
			{
				xSemaphoreGive(xNodeCfgMutex);
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON node name is not a string");
				goto api_post_handler_fail;
			}

			// enabled state
			if (!cJSON_IsBool(json_node_en))
			{
				xSemaphoreGive(xNodeCfgMutex);
				httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "JSON node enabled state is not a bool");
				goto api_post_handler_fail;
			}




			if (json_node_addr->valueint == 0)
			{
				// disable if address is zero
				nodes[i].flag_enabled = 0;
			}
			else
			{
				if (cJSON_IsTrue(json_node_en))
					nodes[i].flag_enabled = 1;
				else
					nodes[i].flag_enabled = 0;
			}


			nodes[i].address = json_node_addr->valueint;
			nodes[i].group = json_node_grp->valueint;
			nodes[i].group_pos = json_node_pos->valueint;


			strncpy(nodes[i].name, json_node_name->valuestring, MAX_NAME);
			nodes[i].name[MAX_NAME-1] = '\0';	// string will not be null terminated if too long, add manually for safety

			i++;


		}

		xSemaphoreGive(xNodeCfgMutex);

		// restart timer
		if (xTimerChangePeriod(tmr_node_refresh, ((uint32_t)netCfg.updInterval * 1000) / portTICK_PERIOD_MS, 2000 / portTICK_PERIOD_MS) == pdFALSE)
		{
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Busy, try again later!");
			goto api_post_handler_fail;
		}

		xTaskNotify(h_comm_task, COMM_REQ_MEASURE, eSetBits);	// trigger measurement collection (no-op if already pending)

		// save to NVS (mutex not needed as the above code is the only place that writes to node cfg)

		ESP_LOGI(TAG, "Trying to save group/node cfg to NVS...");

		nvs_handle_t hNvsCfg;

		ret = nvs_open("cfg", NVS_READWRITE, &hNvsCfg);

		if (ret == ESP_OK)
		{
			// upd interval
			ret = nvs_set_u16(hNvsCfg, "upd_int", netCfg.updInterval);
			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save update interval to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save config to NVS!");
				goto api_post_handler_fail;
			}


			// groups
			size_t nvsGroupBufLen = sizeof(nvs_group_t) * MAX_GROUPS;
			nvs_group_t *nvsGroupBuf = malloc(nvsGroupBufLen);

			if (nvsGroupBuf == NULL)
			{
				nvs_close(hNvsCfg);
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
				goto api_post_handler_fail;
			}


			for (int i=0; i<MAX_GROUPS; i++)
			{
				nvsGroupBuf[i].state = groups[i].state;
				strncpy(nvsGroupBuf[i].name, groups[i].name, MAX_NAME);
				nvsGroupBuf[i].name[MAX_NAME-1] = '\0';	// make sure last char is null for safety
			}


			ret = nvs_set_blob(hNvsCfg, "group_cfg", nvsGroupBuf, nvsGroupBufLen);

			free(nvsGroupBuf);	// not needed anymore

			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save group cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save config to NVS!");
				goto api_post_handler_fail;
			}

			// nodes
			size_t nvsNodeBufLen = sizeof(nvs_node_t) * MAX_NODES;
			nvs_node_t *nvsNodeBuf = malloc(nvsNodeBufLen);

			if (nvsNodeBuf == NULL)
			{
				nvs_close(hNvsCfg);
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory!");
				goto api_post_handler_fail;
			}


			for (int i=0; i<MAX_NODES; i++)
			{
				nvsNodeBuf[i].address = nodes[i].address;
				nvsNodeBuf[i].group = nodes[i].group;
				nvsNodeBuf[i].group_pos = nodes[i].group_pos;
				nvsNodeBuf[i].state = nodes[i].flag_enabled ? NODE_STATE_ENABLED : NODE_STATE_DISABLED;
				strncpy(nvsNodeBuf[i].name, nodes[i].name, MAX_NAME);
				nvsNodeBuf[i].name[MAX_NAME-1] = '\0';	// make sure last char is null for safety
			}

			ret = nvs_set_blob(hNvsCfg, "node_cfg", nvsNodeBuf, nvsNodeBufLen);

			free(nvsNodeBuf);	// not needed anymore

			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save node cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save config to NVS!");
				goto api_post_handler_fail;
			}


			// commit and close

			ret = nvs_commit(hNvsCfg);
			nvs_close(hNvsCfg);

			if (ret != ESP_OK)
			{
				ESP_LOGW(TAG, "Unable to commit cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save config to NVS!");
				goto api_post_handler_fail;
			}

		}
		else
		{
			ESP_LOGW(TAG, "Unable to save group/node cfg to NVS!");
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save config to NVS!");
			goto api_post_handler_fail;
		}





	}
	else if (strcmp(op->valuestring,"getnetcfg") == 0)
	{
		// get network cfg
		char ipStr[16];

		// wifi ssid
		cJSON_AddStringToObject(rootResp, "ssid", (char*)netCfg.wifi_config.sta.ssid);

		// wifi type
		if (netCfg.wifi_config.sta.threshold.authmode == WIFI_AUTH_OPEN)
		{
			cJSON_AddStringToObject(rootResp, "type", "open");
		}
		else if (netCfg.wifi_config.sta.threshold.authmode == WIFI_AUTH_WPA_PSK)
		{
			cJSON_AddStringToObject(rootResp, "type", "wpa-psk");
		}
		else if (netCfg.wifi_config.sta.threshold.authmode == WIFI_AUTH_WPA2_PSK)
		{
			cJSON_AddStringToObject(rootResp, "type", "wpa2-psk");
		}
		else if (netCfg.wifi_config.sta.threshold.authmode == WIFI_AUTH_WPA3_PSK)
		{
			cJSON_AddStringToObject(rootResp, "type", "wpa3-psk");
		}

		// ip
		if (NULL == ip4addr_ntoa_r(&netCfg.ip_addr, ipStr, sizeof(ipStr)))
		{
			cJSON_AddNullToObject(rootResp, "ip");
		}
		else
		{
			cJSON_AddStringToObject(rootResp, "ip", ipStr);
		}

		// mask
		if (NULL == ip4addr_ntoa_r(&netCfg.ip_subnet, ipStr, sizeof(ipStr)))
		{
			cJSON_AddNullToObject(rootResp, "sub");
		}
		else
		{
			cJSON_AddStringToObject(rootResp, "sub", ipStr);
		}

		// gw
		if (NULL == ip4addr_ntoa_r(&netCfg.ip_gw, ipStr, sizeof(ipStr)))
		{
			cJSON_AddNullToObject(rootResp, "gw");
		}
		else
		{
			cJSON_AddStringToObject(rootResp, "gw", ipStr);
		}

		// dns
		if (NULL == ip4addr_ntoa_r(&netCfg.ip_dns, ipStr, sizeof(ipStr)))
		{
			cJSON_AddNullToObject(rootResp, "dns");
		}
		else
		{
			cJSON_AddStringToObject(rootResp, "dns", ipStr);
		}

		// sntp server
		cJSON_AddStringToObject(rootResp, "sntp", netCfg.sntp_srv);

		// timezone name
		cJSON_AddStringToObject(rootResp, "tz", netCfg.timezone.name);

		// timezone
		cJSON_AddStringToObject(rootResp, "tz_raw", netCfg.timezone.tz);





	}
	else if (strcmp(op->valuestring,"setnetcfg") == 0)
	{
		// set network cfg
		//char ipStr[16];
		ip4_addr_t ip, sub, gw, dns;
		//char ssid[32];
		//char pw[64];

		wifi_auth_mode_t authType;

		cJSON *json_node_ssid = cJSON_GetObjectItem(root,"ssid");
		cJSON *json_node_type = cJSON_GetObjectItem(root,"type");
		cJSON *json_node_pw = cJSON_GetObjectItem(root,"pw");
		cJSON *json_node_ip = cJSON_GetObjectItem(root,"ip");
		cJSON *json_node_sub = cJSON_GetObjectItem(root,"sub");
		cJSON *json_node_gw = cJSON_GetObjectItem(root,"gw");
		cJSON *json_node_dns = cJSON_GetObjectItem(root,"dns");
		cJSON *json_node_sntp = cJSON_GetObjectItem(root,"sntp");
		cJSON *json_node_tz = cJSON_GetObjectItem(root,"tz");
		cJSON *json_node_tz_raw = cJSON_GetObjectItem(root,"tz_raw");


		// check ssid
		if ((!cJSON_IsString(json_node_ssid)) || (strlen(json_node_ssid->valuestring) > 31))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid SSID!");
			goto api_post_handler_fail;
		}

		// check type


		if (!cJSON_IsString(json_node_type))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid wifi type!");
			goto api_post_handler_fail;
		}

		if (strcmp(json_node_type->valuestring, "open") == 0)
			authType = WIFI_AUTH_OPEN;
		else if (strcmp(json_node_type->valuestring, "wpa-psk") == 0)
			authType = WIFI_AUTH_WPA_PSK;
		else if (strcmp(json_node_type->valuestring, "wpa2-psk") == 0)
			authType = WIFI_AUTH_WPA2_PSK;
		else if (strcmp(json_node_type->valuestring, "wpa3-psk") == 0)
			authType = WIFI_AUTH_WPA3_PSK;
		else
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid wifi type!");
			goto api_post_handler_fail;
		}


		// check pw
		if ((!cJSON_IsNull(json_node_pw)) && ((!cJSON_IsString(json_node_pw)) || (strlen(json_node_pw->valuestring) < 8) || (strlen(json_node_pw->valuestring) > 63)))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid password!");
			goto api_post_handler_fail;
		}

		// check ip
		if ((!cJSON_IsString(json_node_ip)) || (strlen(json_node_ip->valuestring) > 15) || (!ip4addr_aton(json_node_ip->valuestring, &ip)))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid IP!");
			goto api_post_handler_fail;
		}

		// check subnet mask
		if ((!cJSON_IsString(json_node_sub)) || (strlen(json_node_sub->valuestring) > 15) || (!ip4addr_aton(json_node_sub->valuestring, &sub)) || (!ip4_addr_netmask_valid(sub.addr)))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid subnet mask!");
			goto api_post_handler_fail;
		}

		// check gw
		if ((!cJSON_IsString(json_node_gw)) || (strlen(json_node_gw->valuestring) > 15) || (!ip4addr_aton(json_node_gw->valuestring, &gw)))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid gateway!");
			goto api_post_handler_fail;
		}

		// check dns
		if ((!cJSON_IsString(json_node_dns)) || (strlen(json_node_dns->valuestring) > 15) || (!ip4addr_aton(json_node_dns->valuestring, &dns)))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid DNS!");
			goto api_post_handler_fail;
		}

		// check SNTP
		if ((!cJSON_IsString(json_node_sntp)) || (strlen(json_node_sntp->valuestring) > 63))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid SNTP!");
			goto api_post_handler_fail;
		}

		// check TZ
		if ((!cJSON_IsString(json_node_tz)) || (strlen(json_node_tz->valuestring) > 63))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid time zone!");
			goto api_post_handler_fail;
		}

		// check TZ raw string
		if ((!cJSON_IsString(json_node_tz_raw)) || (strlen(json_node_tz_raw->valuestring) > 63))
		{
			httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid time zone!");
			goto api_post_handler_fail;
		}


		// all checks OK, apply new settings
		strcpy((char*)netCfg.wifi_config.sta.ssid, json_node_ssid->valuestring);



		netCfg.wifi_config.sta.threshold.authmode = authType;
		if (authType == WIFI_AUTH_OPEN)
		{
			// no password
			memset(netCfg.wifi_config.sta.password, 0, sizeof(netCfg.wifi_config.sta.password));	// clear PW
		}
		else if (!cJSON_IsNull(json_node_pw))
		{
			// new password is set
			memset(netCfg.wifi_config.sta.password, 0, sizeof(netCfg.wifi_config.sta.password));	// clear old PW
			strcpy((char*)netCfg.wifi_config.sta.password, json_node_pw->valuestring);
		}

		// IP settings
		netCfg.ip_addr = ip;
		netCfg.ip_subnet = sub;
		netCfg.ip_gw = gw;
		netCfg.ip_dns = dns;


		// time settings
		strcpy(netCfg.sntp_srv, json_node_sntp->valuestring);
		strcpy(netCfg.timezone.name, json_node_tz->valuestring);
		strcpy(netCfg.timezone.tz, json_node_tz_raw->valuestring);

		// save to NVS
		ESP_LOGI(TAG, "Trying to save network cfg to NVS...");

		nvs_handle_t hNvsCfg;

		ret = nvs_open("netcfg", NVS_READWRITE, &hNvsCfg);
		if (ret == ESP_OK)
		{
			// wifi auth type
			ret = nvs_set_u32(hNvsCfg, "wifi_auth", netCfg.wifi_config.sta.threshold.authmode);
			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
				goto api_post_handler_fail;
			}

			// wifi ssid
			ret = nvs_set_str(hNvsCfg, "wifi_ssid", (char*)netCfg.wifi_config.sta.ssid);
			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
				goto api_post_handler_fail;
			}

			// wifi pw
			ret = nvs_set_str(hNvsCfg, "wifi_pw", (char*)netCfg.wifi_config.sta.password);
			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
				goto api_post_handler_fail;
			}

			// ip
			ret = nvs_set_u32(hNvsCfg, "ip_addr", netCfg.ip_addr.addr);
			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
				goto api_post_handler_fail;
			}

			// mask
			ret = nvs_set_u32(hNvsCfg, "ip_subnet", netCfg.ip_subnet.addr);
			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
				goto api_post_handler_fail;
			}

			// gw
			ret = nvs_set_u32(hNvsCfg, "ip_gw", netCfg.ip_gw.addr);
			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
				goto api_post_handler_fail;
			}

			// dns
			ret = nvs_set_u32(hNvsCfg, "ip_dns", netCfg.ip_dns.addr);
			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
				goto api_post_handler_fail;
			}

			// timezone
			ret = nvs_set_str(hNvsCfg, "tz", netCfg.timezone.tz);
			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
				goto api_post_handler_fail;
			}

			// timezone name
			ret = nvs_set_str(hNvsCfg, "tzname", netCfg.timezone.name);
			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
				goto api_post_handler_fail;
			}

			// sntp
			ret = nvs_set_str(hNvsCfg, "sntp", netCfg.sntp_srv);
			if (ret != ESP_OK)
			{
				nvs_close(hNvsCfg);
				ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
				goto api_post_handler_fail;
			}

			// commit and close
			ret = nvs_commit(hNvsCfg);
			nvs_close(hNvsCfg);

			if (ret != ESP_OK)
			{
				ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
				httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
				goto api_post_handler_fail;
			}

		}
		else
		{
			ESP_LOGW(TAG, "Unable to save network cfg to NVS!");
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to save network config to NVS!");
			goto api_post_handler_fail;
		}

		cJSON_AddTrueToObject(rootResp, "status");

	}
	else
	{
		// unknown cmd
		httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown cmd!");
		goto api_post_handler_fail;
	}

	// request JSON not needed anymore, delete
	cJSON_Delete(root);

	// send response

	char *json = NULL;
	json = cJSON_Print(rootResp);
	cJSON_Minify(json);
	httpd_resp_set_type(req, "application/json");
	httpd_resp_sendstr(req, json);
	free(json);
	cJSON_Delete(rootResp);
	free(buf);
	return ESP_OK;

api_post_handler_fail:
	cJSON_Delete(root);
	cJSON_Delete(rootResp);
	free(buf);
	return ESP_FAIL;
}







/*
 * Handle OTA file upload
 */
esp_err_t fwupdate_post_handler(httpd_req_t *req)
{
	char buf[1000];
	esp_ota_handle_t ota_handle;
	int remaining = req->content_len;

	const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
	ESP_ERROR_CHECK(esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle));

	while (remaining > 0) {
		int recv_len = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));

		// Timeout Error: Just retry
		if (recv_len == HTTPD_SOCK_ERR_TIMEOUT) {
			continue;

		// Serious Error: Abort OTA
		} else if (recv_len <= 0) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Protocol Error");
			return ESP_FAIL;
		}

		// Successful Upload: Flash firmware chunk
		if (esp_ota_write(ota_handle, (const void *)buf, recv_len) != ESP_OK) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Flash Error");
			return ESP_FAIL;
		}

		remaining -= recv_len;
	}

	// Validate and switch to new OTA image and reboot
	if (esp_ota_end(ota_handle) != ESP_OK || esp_ota_set_boot_partition(ota_partition) != ESP_OK) {
			httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Validation / Activation Error");
			return ESP_FAIL;
	}

	httpd_resp_sendstr(req, "Firmware update complete, rebooting now!");

	vTaskDelay(500 / portTICK_PERIOD_MS);
	esp_restart();

	return ESP_OK;
}


/*
 * HTTP Server
 */
httpd_uri_t nodefwupdate_post = {
	.uri	  = "/nodefwupdate",
	.method   = HTTP_POST,
	.handler  = nodefwupdate_post_handler,
	.user_ctx = NULL
};

//httpd_uri_t nodefw_do_upd_post = {
//	.uri	  = "/nodefw_do_upd",
//	.method   = HTTP_POST,
//	.handler  = nodefw_do_upd_post_handler,
//	.user_ctx = NULL
//};

httpd_uri_t fwupdate_post = {
	.uri	  = "/fwupdate",
	.method   = HTTP_POST,
	.handler  = fwupdate_post_handler,
	.user_ctx = NULL
};

httpd_uri_t api_get = {
	.uri	  = "/api",
	.method   = HTTP_GET,
	.handler  = api_get_handler,
	.user_ctx = NULL
};

httpd_uri_t api_post = {
	.uri	  = "/api",
	.method   = HTTP_POST,
	.handler  = api_post_handler,
	.user_ctx = NULL
};

httpd_uri_t cfg_get = {
	.uri	  = "/config",
	.method   = HTTP_GET,
	.handler  = cfg_get_handler,
	.user_ctx = NULL
};

httpd_uri_t static_get = {
	.uri	  = "*",
	.method   = HTTP_GET,
	.handler  = static_get_handler,
	.user_ctx = NULL
};

httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.max_uri_handlers = 16;
    config.uri_match_fn = httpd_uri_match_wildcard;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
		httpd_register_uri_handler(server, &fwupdate_post);
		httpd_register_uri_handler(server, &nodefwupdate_post);
//		httpd_register_uri_handler(server, &nodefw_do_upd_post);
		httpd_register_uri_handler(server, &api_get);
		httpd_register_uri_handler(server, &api_post);
		//httpd_register_uri_handler(server, &editnodes_get);
		httpd_register_uri_handler(server, &cfg_get);
		httpd_register_uri_handler(server, &static_get);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}


void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_stop(server);
}
