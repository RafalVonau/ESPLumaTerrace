/*
 * NVRAM (use ESP32 FLASH based NVS key->value emulation).
 *                    ________    ___________   __
 *                   / ____/ /   / ____/  _/ | / /
 * Copyright (C)    / __/ / /   / /_   / //  |/ /
 *                 / /___/ /___/ __/ _/ // /|  /
 *                /_____/_____/_/   /___/_/ |_/
 *
 * Author: Rafal Vonau <rafal.vonau@elfin-pe.pl>
 */
#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "nvram.h"
#include <string>
#include <sstream>
#include <iostream>
#include <algorithm>

#ifdef DEBUG
#define msg_info(fmt, args...) ESP_LOGI("NVRAM", fmt, ## args);
#define msg_init(fmt, args...)  ESP_LOGI("NVRAM", fmt, ## args);
#define msg_debug(fmt, args...)  ESP_LOGI("NVRAM", fmt, ## args);
#define msg_error(fmt, args...)  ESP_LOGE("NVRAM", fmt, ## args);
#else
#define msg_info(fmt, args...)
#define msg_init(fmt, args...)
#define msg_debug(fmt, args...)
#define msg_error(fmt, args...)  ESP_LOGE("NVRAM", fmt, ## args);
#endif

uint32_t SN_int = 0;

/*!
 * \brief Get serial number from NVRAM.
 * \return uint8_t - serial number.
 */
uint8_t nvram_sn_init()
{
	esp_err_t err;
	nvs_handle_t nvs_sn_handle;
	const char *nvs_sn_partition_label = "nvs-sn";
	uint16_t lsn = 0xffff;

	err = nvs_flash_init_partition(nvs_sn_partition_label);
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		msg_debug("Erase nvs-sn partitiion!!");
		nvs_flash_erase_partition(nvs_sn_partition_label);
		err = nvs_flash_init_partition(nvs_sn_partition_label);
	}
	// Open
	err = nvs_open_from_partition(nvs_sn_partition_label,"tenso-sn", NVS_READONLY, &nvs_sn_handle);
	if (err != ESP_OK) {
		msg_debug("nvs-sn Open ERROR! 0x%x\n",err);
		return err;
	}
	// Restore serial number
	nvs_get_u16(nvs_sn_handle,"sn",&lsn);
	SN_int = lsn;
	msg_info("SN = %d", lsn);
	nvs_close(nvs_sn_handle);
	return 0;
}
/* ========================================================================================== */
