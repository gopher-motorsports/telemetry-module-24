/*
 * tm.c
 *
 *  Created on: Dec 2, 2023
 *      Author: jonol
 */

#include "tm.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "GopherCAN.h"
#include "GopherCAN_network.h"
#include <stdbool.h>

extern CAN_HandleTypeDef hcan1;

void tm_init() {
	printf("Go4-24 Telemetry Module\n");

	hcan1.Init.Mode = CAN_MODE_LOOPBACK;
	if(HAL_CAN_Init(&hcan1))
	    tm_fault_handler();

	if(init_can(GCAN0, &hcan1, 1, BXTYPE_MASTER))
		tm_fault_handler();

	printf("initialization complete\n");
}

void tm_taskA() {
    HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
    HAL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
    HAL_GPIO_TogglePin(LED_GSENSE_GPIO_Port, LED_GSENSE_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO0_GPIO_Port, RFD_GPIO0_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO1_GPIO_Port, RFD_GPIO1_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO2_GPIO_Port, RFD_GPIO2_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO3_GPIO_Port, RFD_GPIO3_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO4_GPIO_Port, RFD_GPIO4_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO5_GPIO_Port, RFD_GPIO5_Pin);

    bool sd_card_detect = HAL_GPIO_ReadPin(SDIO_CD_GPIO_Port, SDIO_CD_Pin);
    printf("SD_CD: %d\n", sd_card_detect);

    packetsLogged_ul.data += 1;
    send_parameter((CAN_INFO_STRUCT*)&packetsLogged_ul);

    osDelay(1000);
}

void tm_taskB() {
    printf("slow task\n");
    osDelay(5000);
}

void tm_service_can() {
    service_can_tx(&hcan1);
    service_can_rx_buffer();
    osDelay(1);
}

void GCAN_RxMsgPendingCallback(CAN_HandleTypeDef* hcan, U32 rx_mailbox) {
    printf("(CAN RX) packets logged: %lu\n", packetsLogged_ul.data);

    service_can_rx_hardware(hcan, rx_mailbox);
}

void tm_fault_handler() {
    HAL_GPIO_WritePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    NVIC_SystemReset();
}
