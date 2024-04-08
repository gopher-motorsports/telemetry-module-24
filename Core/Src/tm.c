/*
 * tm.c
 *
 *  Created on: Dec 2, 2023
 *      Author: jonathan
 */

#include <stdbool.h>
#include "tm.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "tm_sd.h"
#include "GopherCAN.h"
#include "GopherCAN_network.h"
#include "tm_data.h"

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern ADC_HandleTypeDef hadc1;
extern RTC_HandleTypeDef hrtc;

extern TM_DBL_BUFFER SD_DB;
extern TM_DBL_BUFFER RADIO_DB;

void tm_init() {
	printf("Go4-24 Telemetry Module\n");

#ifdef TM_DEBUG_CAN_LOOPBACK
	// put CAN in loopback mode
	hcan1.Init.Mode = CAN_MODE_LOOPBACK;
	if (HAL_CAN_Init(&hcan1))
	    tm_fault();

	hcan2.Init.Mode = CAN_MODE_LOOPBACK;
	if (HAL_CAN_Init(&hcan2))
		tm_fault();
#endif

	if (init_can(&hcan1, GCAN0))
		tm_fault();

	if (init_can(&hcan2, GCAN1))
		tm_fault();

	printf("initialization complete\n");
}

void tm_heartbeat() {
	HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);

	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
	printf("20%u-%02u-%02u-%02u-%02u-%02u\n", date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);

	osDelay(TM_DELAY_HEARTBEAT);
}

void tm_service_can() {
    service_can_tx(&hcan1);
    service_can_tx(&hcan2);
    service_can_rx_buffer();

    osDelay(TM_DELAY_SERVICE_CAN);
}

void tm_collect_data() {
//	HAL_ADC_Start(&hadc1);
//	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//	uint32_t raw = HAL_ADC_GetValue(&hadc1);
//	printf("ADC: %ld\n", raw);
//	HAL_ADC_Stop(&hadc1);

	osDelay(TM_DELAY_COLLECT_DATA);
}

void tm_store_data() {
	static bool sd_ready = 0;
	if (!sd_ready) {
		if (tm_sd_init() != TM_OK)
			tm_sd_deinit();
		else
			sd_ready = 1;
	}

	if (sd_ready) {
		if (tm_sd_write((uint8_t*)"test", 4) != TM_OK) {
			sd_ready = 0;
			tm_sd_deinit();
		}
	}

	osDelay(TM_DELAY_STORE_DATA);
}

void tm_transmit_data() {
//	HAL_UART_Transmit(&huart1, (uint8_t*)"test", 4, HAL_MAX_DELAY);

//	packetsLogged_ul.data += 1;
//	send_parameter(PACKETSLOGGED_UL_ID);

	osDelay(TM_DELAY_TRANSMIT_DATA);
}

void tm_fault() {
	vTaskSuspendAll();
	printf("! FAULT !\n");
    HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_SET);
    HAL_Delay(5000);
    HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    NVIC_SystemReset();
}
