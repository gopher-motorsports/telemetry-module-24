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

// buffer for ADC_VBAT samples (coin cell battery)
#define ADC_VBAT_BUF_SIZE 128
uint16_t ADC_VBAT_BUF[ADC_VBAT_BUF_SIZE];

// CAN RX interrupt counters
uint32_t hcan1_rx_count = 0;
uint32_t hcan2_rx_count = 0;

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

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_VBAT_BUF, ADC_VBAT_BUF_SIZE);

	printf("initialization complete\n");
}

void tm_heartbeat() {
	uint32_t tick = HAL_GetTick();
	static uint32_t last_can_calc = 0;

	HAL_GPIO_TogglePin(LED_HEARTBEAT_GPIO_Port, LED_HEARTBEAT_Pin);

	// print time stamp
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);
	printf("20%u-%02u-%02u-%02u-%02u-%02u\n", date.Year, date.Month, date.Date, time.Hours, time.Minutes, time.Seconds);

	// update logging metrics
	tm_SDBufferFill_percent.info.last_rx = tick;
	tm_RadioBufferFill_percent.info.last_rx = tick;
	tm_SDBytesTransferred_bytes.info.last_rx = tick;
	tm_RadioBytesTransferred_bytes.info.last_rx = tick;
	tm_SDPacketsDropped_ul.info.last_rx = tick;
	tm_RadioPacketsDropped_ul.info.last_rx = tick;

	// average coin cell voltage samples
	uint32_t adc_total = 0;
	for (size_t i = 0; i < ADC_VBAT_BUF_SIZE; i++) {
		adc_total += ADC_VBAT_BUF[i];
	}
	float adc_avg = (float) adc_total / ADC_VBAT_BUF_SIZE;
	tm_CoinBattery_V.data = adc_avg / 4096.0f * 3.3f;
	tm_CoinBattery_V.info.last_rx = tick;

	// estimate CAN utilization
	// assumes 1Mbps bus
	// assumes each frame is 44 + 8N (data bytes) + 3 (inter-frame) = 111 bits
	// assumes every frame has 8 bytes of data (worst case)
	// ignores bit stuffing
	float sec_since_last_update = (HAL_GetTick() - last_can_calc) / 1000.0f;
	float hcan1_util = hcan1_rx_count * 111.0f / sec_since_last_update / 1e6;
	float hcan2_util = hcan2_rx_count * 111.0f / sec_since_last_update / 1e6;
	tm_CAN1Util_percent.data = hcan1_util * 100.0f;
	tm_CAN2Util_percent.data = hcan2_util * 100.0f;
	tm_CAN1Util_percent.info.last_rx = tick;
	tm_CAN2Util_percent.info.last_rx = tick;
	last_can_calc = HAL_GetTick();

#ifdef TM_DEBUG_SIMULATE_DATA
	for (uint8_t i = 1; i < NUM_OF_PARAMETERS; i++) {
		send_parameter(i);
	}
#endif

	osDelay(TM_DELAY_HEARTBEAT);
}

void tm_service_can() {
    service_can_tx(&hcan1);
    service_can_tx(&hcan2);
    service_can_rx_buffer();

    osDelay(TM_DELAY_SERVICE_CAN);
}

void GCAN_onRX(CAN_HandleTypeDef* hcan) {
	if (hcan->Instance == CAN1) hcan1_rx_count++;
	else if (hcan->Instance == CAN2) hcan2_rx_count++;
}

void tm_collect_data() {
	static uint32_t sd_last_log[NUM_OF_PARAMETERS] = {0};
	static uint32_t radio_last_tx[NUM_OF_PARAMETERS] = {0};

	for (uint8_t i = 1; i < NUM_OF_PARAMETERS; i++) {
		CAN_INFO_STRUCT* param = PARAMETERS[i];
		uint32_t tick = HAL_GetTick();

		if (param->last_rx > sd_last_log[i]) {
			// parameter has been updated
			// create packet and add to SD buffer
			TM_RES res = tm_data_record(SD_DB.buffers[SD_DB.write_index], param);
			if (res == TM_OK) {
				sd_last_log[i] = tick;
			} else {
				tm_SDPacketsDropped_ul.data += 1;
			}
		}

		if (param->last_rx > radio_last_tx[i] && (tick - radio_last_tx[i]) > TM_RADIO_TX_DELAY) {
			// parameter has been updated and hasn't been sent in a while
			// create packet and add to radio buffer
			TM_RES res = tm_data_record(RADIO_DB.buffers[RADIO_DB.write_index], param);
			if (res == TM_OK) {
				radio_last_tx[i] = tick;
			} else {
				tm_RadioPacketsDropped_ul.data += 1;
			}
		}
	}

	tm_SDBufferFill_percent.data = (float) SD_DB.buffers[SD_DB.write_index]->fill / SD_DB.buffers[SD_DB.write_index]->size * 100.0f;
	tm_RadioBufferFill_percent.data = (float) RADIO_DB.buffers[RADIO_DB.write_index]->fill / RADIO_DB.buffers[RADIO_DB.write_index]->size * 100.0f;

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

	if (sd_ready && !SD_DB.tx_cplt) {
		// FatFs initialized, waiting for a transfer
		TM_BUFFER* buffer = SD_DB.buffers[!SD_DB.write_index];
		if (buffer->fill > 0) {
			if (tm_sd_write(buffer->bytes, buffer->fill) != TM_OK) {
				// write failed
				sd_ready = 0;
				tm_sd_deinit();
			} else {
				// successful write
				SD_DB.tx_cplt = 1;
			}
		} else {
			// nothing to transfer
			SD_DB.tx_cplt = 1;
		}
	}

	// swap buffers after transfer is complete
	// critical section entry/exit is fast and fine for a quick swap
	if (SD_DB.tx_cplt) {
		taskENTER_CRITICAL();
		tm_SDBytesTransferred_bytes.data += SD_DB.buffers[!SD_DB.write_index]->fill;
		SD_DB.buffers[!SD_DB.write_index]->fill = 0;
		SD_DB.write_index = !SD_DB.write_index;
		SD_DB.tx_cplt = 0;
		taskEXIT_CRITICAL();
	}

	osDelay(TM_DELAY_STORE_DATA);
}

void tm_transmit_data() {
	static bool tx_in_progress = false;

	if (!RADIO_DB.tx_cplt && !tx_in_progress) {
		// waiting for a transfer to radio
		TM_BUFFER* buffer = RADIO_DB.buffers[!RADIO_DB.write_index];
		if (buffer->fill > 0) {
			HAL_UART_Transmit_DMA(&huart1, buffer->bytes, buffer->fill);
			tx_in_progress = true;
		} else {
			// nothing to transfer
			RADIO_DB.tx_cplt = 1;
		}
	}

	// swap buffers after transfer is complete
	// critical section entry/exit is fast and fine for a quick swap
	if (RADIO_DB.tx_cplt) {
		taskENTER_CRITICAL();
		tm_RadioBytesTransferred_bytes.data += RADIO_DB.buffers[!RADIO_DB.write_index]->fill;
		RADIO_DB.buffers[!RADIO_DB.write_index]->fill = 0;
		RADIO_DB.write_index = !RADIO_DB.write_index;
		RADIO_DB.tx_cplt = 0;
		tx_in_progress = false;
		taskEXIT_CRITICAL();
	}

	osDelay(TM_DELAY_TRANSMIT_DATA);
}

// triggered when UART DMA transfer is complete
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
    RADIO_DB.tx_cplt = 1;
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
