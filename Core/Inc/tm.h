/*
 * tm.h
 *
 *  Created on: Dec 2, 2023
 *      Author: jonathan
 */

#ifndef INC_TM_H_
#define INC_TM_H_

#define TM_DELAY_HEARTBEAT 1000
#define TM_DELAY_SERVICE_CAN 1
#define TM_DELAY_COLLECT_DATA 1000
#define TM_DELAY_STORE_DATA 1000
#define TM_DELAY_TRANSMIT_DATA 1000

#define TM_DEBUG_CAN_LOOPBACK
#define TM_DEBUG_SIMULATE_DATA

#define TM_RTC_MAGIC 0x24 // used to check if backup register has been cleared
#define TM_RADIO_TX_DELAY 100 // minimum time (ms) until param can be sent again

typedef enum {
    TM_OK     = 0,
    TM_ERR    = 1
} TM_RES;

void tm_init();
void tm_heartbeat();
void tm_service_can();
void tm_collect_data();
void tm_store_data();
void tm_transmit_data();
void tm_fault();

#endif /* INC_TM_H_ */
