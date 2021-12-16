/*
 * cobs.h
 *
 *  Created on: 17 sep. 2021
 *      Author: Mathias
 */

#ifndef MAIN_INC_COBS_H_
#define MAIN_INC_COBS_H_

#include <stdint.h>

bool cobs_decode(uint8_t bufLen, uint8_t *buf);
bool cobs_encode(uint8_t bufLen, uint8_t *buf);

#endif /* MAIN_INC_COBS_H_ */
