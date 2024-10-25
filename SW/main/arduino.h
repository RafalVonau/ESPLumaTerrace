/*
 * Arduino compat.
 *
 * Author: Rafal Vonau <rafal.vonau@gmail.com>
 */
#ifndef __ARDUINO_COMPAT_H_
#define __ARDUINO_COMPAT_H_


#ifdef __cplusplus
extern "C" {
#endif

#define NOP() asm volatile ("nop")
unsigned long IRAM_ATTR micros();
unsigned long IRAM_ATTR millis();
void delay(uint32_t ms);
void IRAM_ATTR delayMicroseconds(uint32_t us);

#define LOW               0x0
#define HIGH              0x1

//GPIO FUNCTIONS
#define INPUT             0x01
#define OUTPUT            0x02
#define PULLUP            0x04
#define INPUT_PULLUP      0x05
#define PULLDOWN          0x08
#define INPUT_PULLDOWN    0x09
#define OPEN_DRAIN        0x10
#define OUTPUT_OPEN_DRAIN 0x12
#define SPECIAL           0xF0
#define FUNCTION_1        0x00
#define FUNCTION_2        0x20
#define FUNCTION_3        0x40
#define FUNCTION_4        0x60
#define FUNCTION_5        0x80
#define FUNCTION_6        0xA0
#define ANALOG            0xC0

//Interrupt Modes
#define DISABLED  0x00
#define RISING    0x01
#define FALLING   0x02
#define CHANGE    0x03
#define ONLOW     0x04
#define ONHIGH    0x05
#define ONLOW_WE  0x0C
#define ONHIGH_WE 0x0D

typedef struct {
    uint8_t reg;      /*!< GPIO register offset from DR_REG_IO_MUX_BASE */
    int8_t rtc;       /*!< RTC GPIO number (-1 if not RTC GPIO pin) */
    int8_t adc;       /*!< ADC Channel number (-1 if not ADC pin) */
    int8_t touch;     /*!< Touch Channel number (-1 if not Touch pin) */
} esp32_gpioMux_t;

#ifdef CONFIG_IDF_TARGET_ESP32  
extern const esp32_gpioMux_t esp32_gpioMux[40];
extern const int8_t esp32_adc2gpio[20];
#endif
#define ESP_REG(addr) *((volatile uint32_t *)(addr))


#define digitalPinIsValid(pin)          ((pin) < 40 && esp32_gpioMux[(pin)].reg)
#define digitalPinCanOutput(pin)        ((pin) < 34 && esp32_gpioMux[(pin)].reg)
#define digitalPinToRtcPin(pin)         (((pin) < 40)?esp32_gpioMux[(pin)].rtc:-1)
#define digitalPinToAnalogChannel(pin)  (((pin) < 40)?esp32_gpioMux[(pin)].adc:-1)
#define digitalPinToTouchChannel(pin)   (((pin) < 40)?esp32_gpioMux[(pin)].touch:-1)
#define digitalPinToDacChannel(pin)     (((pin) == 25)?0:((pin) == 26)?1:-1)

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
#ifdef CONFIG_IDF_TARGET_ESP32  
void digitalWriteMask(uint32_t pinMask, uint8_t val);
#endif
int digitalRead(uint8_t pin);

uint32_t getCpuFrequencyMhz();
uint32_t getXtalFrequencyMhz();

#ifdef __cplusplus
}
#endif


#endif
