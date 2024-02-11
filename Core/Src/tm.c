/*
 * tm.c
 *
 *  Created on: Dec 2, 2023
 *      Author: jonol
 */

#include <stdbool.h>
#include "tm.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "tm_sd.h"
#include "GopherCAN.h"
#include "GopherCAN_network.h"

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;
extern ADC_HandleTypeDef hadc1;
extern RTC_HandleTypeDef hrtc;

void tm_init() {
	printf("Go4-24 Telemetry Module\n");

#ifdef TM_DEBUG
	hcan1.Init.Mode = CAN_MODE_LOOPBACK;
	if (HAL_CAN_Init(&hcan1))
	    tm_fault();
#endif

	if (init_can(&hcan1, GCAN0))
		tm_fault();

	printf("initialization complete\n");
}

void tm_taskA() {
    HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
    HAL_UART_Transmit(&huart1, (uint8_t*)"demo", 4, HAL_MAX_DELAY);
    osDelay(1000);
}

void tm_taskB() {
	HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

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

//    HAL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
//    HAL_GPIO_TogglePin(RFD_GPIO0_GPIO_Port, RFD_GPIO0_Pin);
//    HAL_GPIO_TogglePin(RFD_GPIO1_GPIO_Port, RFD_GPIO1_Pin);
//    HAL_GPIO_TogglePin(RFD_GPIO2_GPIO_Port, RFD_GPIO2_Pin);
//    HAL_GPIO_TogglePin(RFD_GPIO3_GPIO_Port, RFD_GPIO3_Pin);
//    HAL_GPIO_TogglePin(RFD_GPIO4_GPIO_Port, RFD_GPIO4_Pin);
//    HAL_GPIO_TogglePin(RFD_GPIO5_GPIO_Port, RFD_GPIO5_Pin);

//    HAL_UART_Transmit(&huart1, (uint8_t*)"demo", 4, HAL_MAX_DELAY);

//    packetsLogged_ul.data += 1;
//    send_parameter((CAN_INFO_STRUCT*)&packetsLogged_ul);

//    HAL_ADC_Start(&hadc1);
//    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//    uint32_t raw = HAL_ADC_GetValue(&hadc1);
//    printf("ADC: %ld\n", raw);
//    HAL_ADC_Stop(&hadc1);

	RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
	printf("20%u-%02u-%02u-%02u-%02u-%02u\n", date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);

    osDelay(5000);
}

void tm_service_can() {
    service_can_tx(&hcan1);
    service_can_rx_buffer();
    osDelay(1);
}

void tm_fault() {
    HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_SET);
    HAL_Delay(5000);
    HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    NVIC_SystemReset();
}
