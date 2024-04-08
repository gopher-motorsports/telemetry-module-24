/*
 * tm_data.h
 *
 *  Created on: Apr 8, 2024
 *      Author: jonathan
 */

#ifndef INC_TM_DATA_H_
#define INC_TM_DATA_H_

#include "tm.h"
#include "GopherCAN.h"
#include <stdbool.h>

// packet control bytes
#define START_BYTE 0x7e // start of packet
#define ESCAPE_BYTE 0x7d // next byte is escaped
#define ESCAPE_XOR 0x20 // escape code

#define TM_SD_BUFFER_SIZE 64000
#define TM_RADIO_BUFFER_SIZE 64000

typedef struct {
    uint8_t* bytes;
    size_t size;
    size_t fill;
} TM_BUFFER;

typedef struct {
    TM_BUFFER* buffers[2];
    uint8_t write_index; // 0 or 1, index of write buffer
    bool tx_cplt; // flag set when transfer of read buffer is complete
} TM_DBL_BUFFER;

TM_RES tm_data_record(TM_BUFFER* buffer, GCAN_PARAM_ID pid);

#endif /* INC_TM_DATA_H_ */
