/*
 * cobs.h
 *
 *  Created on: 4 sep. 2021
 *      Author: Mathias
 */

#ifndef INC_COBS_H_
#define INC_COBS_H_

bool cobs_decode(uint8_t bufLen, uint8_t xdata *buf);
bool cobs_encode(uint8_t bufLen, uint8_t xdata *buf);

#endif /* INC_COBS_H_ */
