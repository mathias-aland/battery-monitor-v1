/*
 * websrv.h
 *
 *  Created on: 8 dec. 2021
 *      Author: Mathias
 */

#ifndef MAIN_INC_WEBSRV_H_
#define MAIN_INC_WEBSRV_H_

#include <esp_http_server.h>

httpd_handle_t start_webserver(void);
void stop_webserver(httpd_handle_t server);

#endif /* MAIN_INC_WEBSRV_H_ */
