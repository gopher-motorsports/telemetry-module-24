/*
 * tm.c
 *
 *  Created on: Dec 2, 2023
 *      Author: jonol
 */

#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"

void tm_init() {

}

void tm_task() {
    HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);
    osDelay(1000);
}
