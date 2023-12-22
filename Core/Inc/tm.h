/*
 * tm.h
 *
 *  Created on: Dec 2, 2023
 *      Author: jonol
 */

#ifndef INC_TM_H_
#define INC_TM_H_

typedef enum {
    TM_OK     = 0,
    TM_ERR    = 1
} TM_RES;

void tm_init();
void tm_taskA();
void tm_taskB();
void tm_service_can();
void tm_fault();

#endif /* INC_TM_H_ */
