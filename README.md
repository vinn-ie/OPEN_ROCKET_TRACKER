# OPEN_ROCKET_TRACKER

Open source, low cost, long range (15km) GPS tracker for rocketry. Uses a 4 layer board designed in Fusion 360 Electronics. Designed to fit in the smallest rockets at only 20mm x 40mm board size (plus antenna and battery you use). 

No radio licence required within Australia. A second board can be used as a base station to log position data via a serial input on a laptop. 

![image](https://github.com/roboticsmick/LOGICOMA_LOW_COST_ROCKET_TRACKER/assets/70121687/0a228a9a-4336-422a-8d21-9c58338da366)

CORE: WIO-E5 LoRa MODULE 
* CPU architecture: STM32WL Cortex M4 32 bit @ 48MHz
* CPU flash memory: 256KB 
* SRAM: 64KB
* LoRa radio: Semtech SX1262 (915 MHz)

GPS: UBlox MAX-M10Q
* Max Altitude: 80,000m
* Max G: ≤4
* Max Velocity: 500m/s
* Velocity Accuracy: 0.05m/s
* Heading Accuracy: 0.3 degrees

Altitude Pressure Sensor: TE Connectivity MS5611
* Operating Pressure: 1kPa ~ 120kPa
* Accuracy: ±0.15kPa
* Operating Temperature: -40°C ~ 85°C

Power systems:
3.3V battery with reverse polarity and ESD protection.
USB-C battery charging.

To do:
* Implement in Zephyr RTOS.
* Design low cost base station with SD card logger, and LCD screen to output GPS coordinates, and direction to rocket position usings GPS and compass.
* Make a simple app for monitoring serial data
