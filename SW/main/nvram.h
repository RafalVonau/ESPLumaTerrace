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
#ifndef __NVRAM_H__
#define __NVRAM_H__

#include <stdint.h>
#include <string>

extern uint32_t SN_int;

uint8_t nvram_sn_init();


#endif
