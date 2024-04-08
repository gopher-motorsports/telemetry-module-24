/*
 * tm_data.c
 *
 *  Created on: Apr 8, 2024
 *      Author: jonathan
 */

#include "tm_data.h"

// SD DOUBLE BUFFER
static uint8_t b1[TM_SD_BUFFER_SIZE];
static TM_BUFFER buffer1 = {
    .bytes = b1,
    .size = TM_SD_BUFFER_SIZE,
    .fill = 0
};

static uint8_t b2[TM_SD_BUFFER_SIZE];
static TM_BUFFER buffer2 = {
    .bytes = b2,
    .size = TM_SD_BUFFER_SIZE,
    .fill = 0
};

TM_DBL_BUFFER SD_DB = {
    .buffers = { &buffer1, &buffer2 },
    .write_index = 0,
    .tx_cplt = 1
};

// RADIO DOUBLE BUFFER
static uint8_t b3[TM_RADIO_BUFFER_SIZE];
static TM_BUFFER buffer3 = {
    .bytes = b3,
    .size = TM_RADIO_BUFFER_SIZE,
    .fill = 0
};

static uint8_t b4[TM_RADIO_BUFFER_SIZE];
static TM_BUFFER buffer4 = {
    .bytes = b4,
    .size = TM_RADIO_BUFFER_SIZE,
    .fill = 0
};

TM_DBL_BUFFER RADIO_DB = {
    .buffers = { &buffer3, &buffer4 },
    .write_index = 0,
    .tx_cplt = 1
};

static void append_byte(TM_BUFFER* buffer, uint8_t byte);

TM_RES tm_data_record(TM_BUFFER* buffer, GCAN_PARAM_ID pid) {
	CAN_INFO_STRUCT* param = PARAMETERS[pid];

    uint32_t timestamp = param->last_rx;
    uint16_t id = param->ID;
    void* data = NULL;
    uint8_t checksum = 0;

    // make sure packet will fit
    uint8_t packet_size = 1 + sizeof(timestamp) + sizeof(id) + param->SIZE + sizeof(checksum);
    if (packet_size * 2 > buffer->size - buffer->fill)
        return TM_ERR;

    // get pointer to data
    switch (param->TYPE) {
        case UNSIGNED8:
            data = &((U8_CAN_STRUCT*)param)->data;
            break;
        case UNSIGNED16:
            data = &((U16_CAN_STRUCT*)param)->data;
            break;
        case UNSIGNED32:
            data = &((U32_CAN_STRUCT*)param)->data;
            break;
        case UNSIGNED64:
            data = &((U64_CAN_STRUCT*)param)->data;
            break;
        case SIGNED8:
            data = &((S8_CAN_STRUCT*)param)->data;
            break;
        case SIGNED16:
            data = &((S16_CAN_STRUCT*)param)->data;
            break;
        case SIGNED32:
            data = &((S32_CAN_STRUCT*)param)->data;
            break;
        case SIGNED64:
            data = &((S64_CAN_STRUCT*)param)->data;
            break;
        case FLOATING:
            data = &((FLOAT_CAN_STRUCT*)param)->data;
            break;
        default:
            return TM_ERR;
    }

    // begin writing packet to buffer
    buffer->bytes[buffer->fill++] = START_BYTE;
    checksum += START_BYTE;

    // append components with MSB first
    U8* bytes;
	U8 i;

	bytes = (U8*) &(timestamp);
	for (i = sizeof(timestamp); i > 0; i--)
	{
		append_byte(buffer, bytes[i - 1]);
		checksum += bytes[i - 1];
	}

	bytes = (U8*) &(id);
	for (i = sizeof(id); i > 0; i--)
	{
		append_byte(buffer, bytes[i - 1]);
		checksum += bytes[i - 1];
	}

	bytes = (U8*) data;
	for (i = param->SIZE; i > 0; i--)
	{
		append_byte(buffer, bytes[i - 1]);
		checksum += bytes[i - 1];
	}

	append_byte(buffer, checksum);

    return TM_OK;
}

static void append_byte(TM_BUFFER *buffer, uint8_t byte) {
    // check for a control byte
    if (byte == START_BYTE || byte == ESCAPE_BYTE) {
        // append escape byte
        buffer->bytes[buffer->fill++] = ESCAPE_BYTE;
        // append the desired byte, escaped
        buffer->bytes[buffer->fill++] = byte ^ ESCAPE_XOR;
    } else {
        // append the raw byte
        buffer->bytes[buffer->fill++] = byte;
    }
}
