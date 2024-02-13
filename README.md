# LOGICOMA_LOW_COST_ROCKET_TRACKER

Open source, low cost, long range (15km) GPS tracker for rocketry. Only 22mm x 48mm. 

No radio licence required within Australia. A second board can be used as a base station to log position data. 

![image](https://github.com/roboticsmick/LOGICOMA_LOW_COST_ROCKET_TRACKER/assets/70121687/ee3f8654-62f5-480b-a4b7-4b1300b47514)

CORE: MAMWLE-02 
* CPU architecture: STM32WL Cortex M4 32 bit @ 48MHz
* CPU flash memory: 256KB 
* SRAM: 64KB
* LoRa radio: Semtech SX1262 (915 MHz)
* Programming: ST-Link 6 pin micro breakout design

![image](https://github.com/roboticsmick/LOGICOMA_LOW_COST_ROCKET_TRACKER/assets/70121687/19545ffb-45a1-46d6-b621-122c30913809)

GPS: UBlox MAX-M10S
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
Implement in Zephyr RTOS.
Build a low cost base station with LCD screen to output GPS coordinates, and direction to position usings GPS and compass.
