/*
 * tm_sd.c
 *
 *  Created on: Dec 7, 2023
 *      Author: jonathan
 */

#include <stdbool.h>
#include "tm_sd.h"
#include "tm.h"
#include "main.h"
#include "fatfs.h"
#include "stm32f4xx_hal.h"

extern RTC_HandleTypeDef hrtc;

TM_RES tm_sd_init() {
    bool sd_detected = HAL_GPIO_ReadPin(SDIO_CD_GPIO_Port, SDIO_CD_Pin) == GPIO_PIN_RESET;
    if (!sd_detected) return TM_ERR;

    printf("initializing SD card...\n");

	if (f_mount(&SDFatFS, SDPath, 1) != FR_OK)
		return TM_ERR;

	// must call GetTime() before GetDate() according to HAL docs
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

	char filename[] = "YYYY-MM-DD-hh-mm-ss.gdat";
	sprintf(filename, "20%u-%02u-%02u-%02u-%02u-%02u.gdat", date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);

    if (f_open(&SDFile, filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK)
        return TM_ERR;

    printf("opened: %s\n", filename);

    if (f_printf(&SDFile, "/%s:\n", filename) <= 0)
        return TM_ERR;

    return TM_OK;
}

void tm_sd_deinit() {
    f_close(&SDFile);
    f_mount(NULL, SDPath, 0);
}

TM_RES tm_sd_write(uint8_t* buffer, uint16_t size) {
    unsigned int bytes_written = 0;
    if (f_write(&SDFile, buffer, size, &bytes_written) != FR_OK)
    	return TM_ERR;

    if (f_sync(&SDFile) != FR_OK)
        return TM_ERR;

    return TM_OK;
}
