<img src="assets/IIA.svg" width=200 align="right">

[![PlatformIO](https://github.com/domino4com/IIA/actions/workflows/platformio.yml/badge.svg)](https://github.com/domino4com/IIA/actions/workflows/platformio.yml)
[![Arduino](https://github.com/domino4com/IIA/actions/workflows/arduino.yml/badge.svg)](https://github.com/domino4com/IIA/actions/workflows/arduino.yml)

# IIA - Input IMU Accelerometer
This is a part of an Inertial measurement unit measuring Acceleration in X, Y and Z axis.

| Specifications | | |
| --: | :--: |:--: |
| Communication | I²C | I²C |
| I²C Address | 0x19 | 0x0F |
| ChipSet | STMicroelectronics LIS2DH12| Rohm Kionix KXTJ3-1057 |
| Datasheet | [.pdf](https://https://www.st.com/resource/en/datasheet/lis2dh12.pdf) |[.pdf](https://www.mouser.com/datasheet/2/348/rohm_s_a0002904669_1-2281852.pdf)|
| Suggested Arduino Library | [GitHub](https://github.com/sparkfun/SparkFun_LIS2DH12_Arduino_Library) |[GitHub](https://github.com/ldab/KXTJ3-1057)|
| Suggested MicroPython Library | [GitHub](https://github.com/electronut/Electronutlabs_CircuitPython_LIS2DH12)||
| Range | ±2g/±4g/±8g/±16g | ±2g/±4g/±8g/±16g |

| Supported I²C Modes | STMicroelectronics LIS2DH12| Rohm Kionix KXTJ3-1057 |
| --: | :--: |:--: |
|100 kbit/s Standard Mode (SM) | :ballot_box_with_check: |:ballot_box_with_check:|
|400 kbit/s	Fast Mode	FM|:ballot_box_with_check:|:ballot_box_with_check:|
| 1 Mbit/s	Fast Mode Plus	FM+|||
|3.4Mbit/s	High Speed Mode	HS||:ballot_box_with_check:|
| 5 Mbit/s	Ultra Fast Mode	UFM|||

# License: 
<img src="assets/CC-BY-NC-SA.svg" width=200 align="right">
Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License

[View License Deed](https://creativecommons.org/licenses/by-nc-sa/4.0/) | [View Legal Code](https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode)
