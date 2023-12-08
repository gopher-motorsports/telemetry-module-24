/*
 * tm_sd.h
 *
 *  Created on: Dec 7, 2023
 *      Author: jonol
 */

#ifndef INC_TM_SD_H_
#define INC_TM_SD_H_

#include "tm.h"

TM_RES tm_sd_init();
void tm_sd_deinit();
TM_RES tm_sd_write(uint8_t* buffer, uint16_t size);

#endif /* INC_TM_SD_H_ */
