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
//#include "gopher_sense.h"
#include "bsp_driver_sd.h"

extern UART_HandleTypeDef huart1;
extern CAN_HandleTypeDef hcan1;
extern ADC_HandleTypeDef hadc1;
extern RTC_HandleTypeDef hrtc;

void tm_init() {
	printf("Go4-24 Telemetry Module\n");

//	hcan1.Init.Mode = CAN_MODE_LOOPBACK;
//	if (HAL_CAN_Init(&hcan1))
//	    tm_fault();

//	if (init_can(GCAN0, &hcan1, 1, BXTYPE_MASTER))
//		tm_fault();

//	if (gsense_init(&hcan1, &hadc1, NULL, NULL, LED_GSENSE_GPIO_Port, LED_GSENSE_Pin))
//	    tm_fault();

//	if (BSP_SD_Init() != MSD_OK) // temporary to test SD card
//        tm_fault();

	printf("initialization complete\n");
}

void tm_taskA() {
	printf("task A\n");

//    HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
//    HAL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
//    HAL_GPIO_TogglePin(LED_GSENSE_GPIO_Port, LED_GSENSE_Pin);
//    HAL_GPIO_TogglePin(RFD_GPIO0_GPIO_Port, RFD_GPIO0_Pin);
//    HAL_GPIO_TogglePin(RFD_GPIO1_GPIO_Port, RFD_GPIO1_Pin);
//    HAL_GPIO_TogglePin(RFD_GPIO2_GPIO_Port, RFD_GPIO2_Pin);
//    HAL_GPIO_TogglePin(RFD_GPIO3_GPIO_Port, RFD_GPIO3_Pin);
//    HAL_GPIO_WritePin(RFD_GPIO3_GPIO_Port, RFD_GPIO3_Pin, GPIO_PIN_SET);
//    HAL_GPIO_TogglePin(RFD_GPIO4_GPIO_Port, RFD_GPIO4_Pin);
//    HAL_GPIO_TogglePin(RFD_GPIO5_GPIO_Port, RFD_GPIO5_Pin);

    osDelay(1000);
}

void tm_taskB() {
    printf("task B\n");

//    static bool sd_ready = 0;
//    if (!sd_ready) {
//        if (tm_sd_init() != TM_OK)
//            tm_sd_deinit();
//        else
//            sd_ready = 1;
//    }

//    if (sd_ready) {
//        if (plm_sd_write("test", 4) != PLM_OK) {
//            // write failed
//            sd_ready = 0;
//            plm_sd_deinit();
//        }
//    }

//    HAL_UART_Transmit(&huart1, "hello", 5, HAL_MAX_DELAY);

//    packetsLogged_ul.data += 1;
//    send_parameter((CAN_INFO_STRUCT*)&packetsLogged_ul);
//    send_parameter((CAN_INFO_STRUCT*)&plmVbatVoltage_V);

//    HAL_ADC_Start(&hadc1);
//    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//    uint32_t raw = HAL_ADC_GetValue(&hadc1);
//    printf("ADC: %ld\n", raw);
//    HAL_ADC_Stop(&hadc1);

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
    HAL_Delay(1000);
    HAL_GPIO_WritePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    NVIC_SystemReset();
}

void GCAN_RxMsgPendingCallback(CAN_HandleTypeDef* hcan, U32 rx_mailbox) {
//    printf("(CAN RX) packets logged: %lu\n", packetsLogged_ul.data);
    service_can_rx_hardware(hcan, rx_mailbox);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
//	HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
//	printf("RTC alarm\n");
//
//	// increment and reset alarm
//	RTC_AlarmTypeDef sAlarm;
//	HAL_RTC_GetAlarm(hrtc, &sAlarm, RTC_ALARM_A, FORMAT_BIN);
//
//	if (sAlarm.AlarmTime.Seconds > 58)
//		sAlarm.AlarmTime.Seconds = 0;
//	else
//		sAlarm.AlarmTime.Seconds += 5;
//
//	while (HAL_RTC_SetAlarm_IT(hrtc, &sAlarm, FORMAT_BIN) != HAL_OK) {}
}
