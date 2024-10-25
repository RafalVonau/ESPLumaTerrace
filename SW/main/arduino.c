#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "soc/uart_reg.h"
#include "soc/timer_group_reg.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "arduino.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "esp_attr.h"
#if (ESP_IDF_VERSION_MAJOR >4)
#else
#include "esp_intr.h"
#endif
#include "rom/gpio.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_struct.h"
#ifdef CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/rtc.h"
#include "esp32/rom/ets_sys.h"
#include "soc/rtc_io_reg.h"
#endif
#include "driver/rtc_io.h"
#include "soc/rtc.h"
//#include "soc/soc_caps.h"
#include <freertos/FreeRTOS.h>
#include <driver/i2s.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <inttypes.h>
#include <string.h>
#include <math.h>
#include "esp_system.h"

#define NOP() asm volatile ("nop")

unsigned long IRAM_ATTR micros()
{
	return (unsigned long) (esp_timer_get_time());
}

unsigned long IRAM_ATTR millis()
{
	return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

void delay(uint32_t ms)
{
	vTaskDelay(ms / portTICK_PERIOD_MS);
}

void IRAM_ATTR delayMicroseconds(uint32_t us)
{
    uint32_t m = micros();
    if(us){
        uint32_t e = (m + us);
        if (m > e){ //overflow
            while(micros() > e){
                NOP();
            }
        }
        while(micros() < e){
            NOP();
        }
    }
}

#ifdef CONFIG_IDF_TARGET_ESP32  

const int8_t esp32_adc2gpio[20] = {36, 37, 38, 39, 32, 33, 34, 35, -1, -1, 4, 0, 2, 15, 13, 12, 14, 27, 25, 26};

const DRAM_ATTR esp32_gpioMux_t esp32_gpioMux[GPIO_PIN_COUNT]={
    {0x44, 11, 11, 1},
    {0x88, -1, -1, -1},
    {0x40, 12, 12, 2},
    {0x84, -1, -1, -1},
    {0x48, 10, 10, 0},
    {0x6c, -1, -1, -1},
    {0x60, -1, -1, -1},
    {0x64, -1, -1, -1},
    {0x68, -1, -1, -1},
    {0x54, -1, -1, -1},
    {0x58, -1, -1, -1},
    {0x5c, -1, -1, -1},
    {0x34, 15, 15, 5},
    {0x38, 14, 14, 4},
    {0x30, 16, 16, 6},
    {0x3c, 13, 13, 3},
    {0x4c, -1, -1, -1},
    {0x50, -1, -1, -1},
    {0x70, -1, -1, -1},
    {0x74, -1, -1, -1},
    {0x78, -1, -1, -1},
    {0x7c, -1, -1, -1},
    {0x80, -1, -1, -1},
    {0x8c, -1, -1, -1},
    {0, -1, -1, -1},
    {0x24, 6, 18, -1}, //DAC1
    {0x28, 7, 19, -1}, //DAC2
    {0x2c, 17, 17, 7},
    {0, -1, -1, -1},
    {0, -1, -1, -1},
    {0, -1, -1, -1},
    {0, -1, -1, -1},
    {0x1c, 9, 4, 8},
    {0x20, 8, 5, 9},
    {0x14, 4, 6, -1},
    {0x18, 5, 7, -1},
    {0x04, 0, 0, -1},
    {0x08, 1, 1, -1},
    {0x0c, 2, 2, -1},
    {0x10, 3, 3, -1}
};

typedef void (*voidFuncPtr)(void);
typedef void (*voidFuncPtrArg)(void*);
typedef struct {
    voidFuncPtr fn;
    void* arg;
    bool functional;
} InterruptHandle_t;
static InterruptHandle_t __pinInterruptHandlers[GPIO_PIN_COUNT] = {0,};

#endif

#if (ESP_IDF_VERSION_MAJOR >4)
#include "esp_chip_info.h"
#include "esp_clk_tree.h"
#include "hal/clk_tree_hal.h"
uint32_t getCpuFrequencyMhz(){ return clk_hal_cpu_get_freq_hz()/1000000u; }
uint32_t getXtalFrequencyMhz() { return clk_hal_xtal_get_freq_mhz(); }
#else
#include "driver/rtc_io.h"
uint32_t getCpuFrequencyMhz(){rtc_cpu_freq_config_t conf; rtc_clk_cpu_freq_get_config(&conf); return conf.freq_mhz;}
uint32_t getXtalFrequencyMhz() { return rtc_clk_xtal_freq_get(); }
#endif

void IRAM_ATTR pinMode(uint8_t pin, uint8_t mode)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;

    if(mode & INPUT) {
        io_conf.mode = GPIO_MODE_INPUT;
    } else if(mode & OUTPUT) {
        io_conf.mode = GPIO_MODE_OUTPUT;
    }
    io_conf.pin_bit_mask = ((uint64_t)1)<<pin;
    io_conf.pull_down_en = (mode & PULLDOWN)?GPIO_PULLDOWN_ENABLE:GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = (mode & PULLUP)?GPIO_PULLUP_ENABLE:GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    if(mode & OUTPUT) {
        gpio_set_direction((gpio_num_t)pin,GPIO_MODE_OUTPUT);
    }

}

void IRAM_ATTR digitalWrite(uint8_t pin, uint8_t val)
{
    gpio_set_level((gpio_num_t)pin, val);
}

#ifdef CONFIG_IDF_TARGET_ESP32    

void IRAM_ATTR digitalWriteMask(uint32_t pinMask, uint8_t val)
{
    if (val == LOW) {
        GPIO.out_w1ts = pinMask;
    } else {
        GPIO.out_w1tc = pinMask;
    }
}

#endif

int IRAM_ATTR digitalRead(uint8_t pin)
{
    return gpio_get_level((gpio_num_t)pin);
}
            

