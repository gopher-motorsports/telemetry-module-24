/*
 * tm_sd.c
 *
 *  Created on: Dec 7, 2023
 *      Author: jonol
 */

#include "tm.h"
#include "fatfs.h"

TM_RES tm_sd_init() {
    printf("initializing SD card...\n");

    if (f_mount(&SDFatFS, SDPath, 1) != FR_OK)
        return TM_ERR;

    char* filename = "data.gdat";
    if (f_open(&SDFile, filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
        return TM_ERR;

    if (f_printf(&SDFile, "/%s:\n", filename) <= 0)
        return TM_ERR;

    return TM_OK;
}

void tm_sd_deinit() {
    printf("deinitializing SD card...\n");

    f_close(&SDFile);
    f_mount(NULL, SDPath, 0);
}

TM_RES tm_sd_write(uint8_t* buffer, uint16_t size) {
    unsigned int bytes_written = 0;
    FRESULT res = f_write(&SDFile, buffer, size, &bytes_written);
    if (res != FR_OK || bytes_written != size) return TM_ERR;

    if (f_sync(&SDFile) != FR_OK)
        return TM_ERR;

    return TM_OK;
}
