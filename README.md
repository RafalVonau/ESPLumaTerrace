# ESP32 Terrace LED Light Controller

The Terrace LED Light Controller is a smart lighting system powered by the ESP32, designed specifically for outdoor terraces and decks. It allows users to create a cozy or vibrant atmosphere with customizable LED lighting.

## Features

- **Sunrise and Sunset Synchronization**: Lights automatically turn on at sunset and off at sunrise, making it convenient and energy-efficient.
- **Energy Efficiency**: Built to save energy by syncing with natural daylight and minimizing power usage.

## Technical Overview

This controller uses the ESP32’s Wi-Fi capabilities to sync time and location data for accurate sunrise and sunset timings.

### Hardware Requirements


### Software Requirements

- **ESP-IDF/platformio Framework** for ESP32 development
- **Wi-Fi Network** for time sync features

## Installation

1. Clone this repository and configure the project for your ESP32 board.
2. Install the ESP-IDF/platformio development environment.
3. Configure Wi-Fi, timezone settings and GPS position in the code.
4. Build and flash the firmware to your ESP32.

## Usage

Once installed, the controller will:
1. Automatically sync time via NTP for accurate sunrise and sunset timings.
2. Control the LED strip based on the local sunrise and sunset.

## Future Improvements

