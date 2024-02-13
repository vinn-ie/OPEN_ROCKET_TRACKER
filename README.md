# LOGICOMA_LOW_COST_ROCKET_TRACKER

Open source, low cost, long range (15km) GPS tracker for rocketry. 

No radio licence required within Australia. A second board can be used as a base station to log position data. 

![image](https://github.com/roboticsmick/LOGICOMA_LOW_COST_ROCKET_TRACKER/assets/70121687/ee3f8654-62f5-480b-a4b7-4b1300b47514)

CORE: MAMWLE-02 
* CPU architecture: STM32WL Cortex M4 32 bit @ 48MHz
* CPU flash memory: 256KB 
* SRAM: 64KB
* LoRa radio: Semtech SX1262 (915 MHz)

![image](https://github.com/roboticsmick/LOGICOMA_LOW_COST_ROCKET_TRACKER/assets/70121687/8808fd6a-6d60-4f33-87eb-f937c80449bb)

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

To do:
Implement in Zephyr RTOS.
Build a low cost base station with LCD screen to output GPS coordinates, and direction to position usings GPS and compass.
