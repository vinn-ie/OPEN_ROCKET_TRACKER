# OPEN_ROCKET_TRACKER

Open source, low cost, long range (15km) GPS tracker for rocketry. Uses a 4 layer board designed in Fusion 360 Electronics. Designed to fit in the smallest rockets at only 20mm x 40mm board size (plus antenna and battery you use). 

No radio licence required within Australia. A second board can be used as a base station to log position data via a serial input on a laptop. 

![image](https://github.com/roboticsmick/OPEN_ROCKET_TRACKER/assets/70121687/0f4b6a38-8444-4cc8-9088-4b34cedbe52a)

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

## Programming the Wio-E5 STM32WLE5JC Module

Connect debug pins to (STLINK-V3MINIE)[https://www.st.com/resource/en/user_manual/um2910-stlinkv3minie-debuggerprogrammer-tiny-probe-for-stm32-microcontrollers-stmicroelectronics.pdf]

| STM32WLE5JC | STLINK | 
|---|---|
| DIO | SWDIO / TMS / STDC14 PIN 4 / MIPI10 PIN 2 |
| CLK | SWCLK / CLK / STDC14 PIN 6 / MIPI10 PIN 4 |
| GND | GND / GND / STDC14 PIN 7 / MIPI10 PIN 5 |
| RST | RST / T_NRST / STDC14 PIN 12 / MIPI10 PIN 10 |
| 3V3 | VCC / T_VCC / STDC14 PIN 3 / MIPI10 PIN 1 |

Choose ST-LINK and set Reset Mode -> hardware reset -> Connect
Open the OB tab -> Change RDP" (Read-out Protection) to AA -> Apply

Add the following board URL (under File / Preferences / Additional board URLs):

```txt
https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
```
Under menu Tools/Board/Boards manager, search for STM32 and install 'STM32 MCU based boards'

Select the board: Tools -> Board -> STM32 MCU Based Boards -> LoRa board
Select the board number: Tools -> Boards part number -> Lora-E5-mini

