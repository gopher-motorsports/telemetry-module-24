/*
 * tm.c
 *
 *  Created on: Dec 2, 2023
 *      Author: jonol
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"
#include "GopherCAN.h"

extern CAN_HandleTypeDef hcan1;

void tm_init() {
	printf("Go4-24 Telemetry Module\n");

	if(init_can(GCAN0, &hcan1, PLM_ID, BXTYPE_MASTER)) {
		HAL_Delay(1000);
		NVIC_SystemReset();
	}

	printf("initialization complete\n");
}

void tm_task() {
    HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
    HAL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
    HAL_GPIO_TogglePin(LED_GSENSE_GPIO_Port, LED_GSENSE_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO0_GPIO_Port, RFD_GPIO0_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO1_GPIO_Port, RFD_GPIO1_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO2_GPIO_Port, RFD_GPIO2_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO3_GPIO_Port, RFD_GPIO3_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO4_GPIO_Port, RFD_GPIO4_Pin);
    HAL_GPIO_TogglePin(RFD_GPIO5_GPIO_Port, RFD_GPIO5_Pin);
    GPIO_PinState sd_card_detect = HAL_GPIO_ReadPin(SDIO_CD_GPIO_Port, SDIO_CD_Pin);

    osDelay(1000);
}
