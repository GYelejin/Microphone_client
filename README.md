# ADC Single Read with TCP Soket

This example demonstrates the following:

- How to obtain a single ADC reading from a GPIO pin using the ADC Driver's Single Read function
- How to use the ADC Calibration functions to obtain a calibrated result (in mV)

## How to use 

### Hardware Required

* A development board with ESP32 SoC (e.g., ESP32-DevKitC, ESP-WROVER-KIT, etc.)
* A USB cable for power supply and programming

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.
