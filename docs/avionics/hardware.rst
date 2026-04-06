Flight Computer Hardware
========================

The Project Viikate Flight Computer is a state-of-the-art flight computer
develeoped around the iMXRT1062-based Teensy 4.1 development board.

The flight computer features:

* a 6-DOF inertial measurement unit (STMicroelectronics ISM330DHCX),
* a high precision magnetometer (MEMSIC MMC5983MA),
* a barometer for altitude sensing (Bosch BME280),
* 1 pyrotechnic ignition channel with continuity sensing in series with an arming switch connection terminal,
* 4 power and PWM channels for servo control (with included level shifter),
* a 10A switching power regulator for translating the 2S LiPo battery input to 5V,
* and optional UART and I2C connection headers for future extensibility.

--------------------
Pin connection table
--------------------

==================  ======================================
Pin Number [#f1]_   Function
==================  ======================================
0                   NC [#f2]_
1                   NC [#f2]_
2                   Servo 3 PWM output
3                   Servo 2 PWM output
4                   Servo 1 PWM output
5                   Servo 0 PWM output
6                   Pyro channel 0 fire
7                   NC [#f2]_
8                   NC [#f2]_
9                   NC [#f2]_
10                  IMU SPI CS
11                  IMU SPI MOSI
12                  IMU SPI MISO
13                  IMU SPI SCK
14                  NC [#f2]_
15                  NC [#f2]_
16                  NC [#f2]_
17                  Magnetometer data ready interrupt
18                  Barometer/Magnetometer I2C SDA
19                  Barometer/Magnetometer I2C SCL
20                  NC [#f2]_
21                  NC [#f2]_
22                  NC [#f2]_
23                  NC [#f2]_
24                  External I2C interface header SCL
25                  External I2C interface header SDA
26                  NC [#f2]_
27                  Pyro channel 0 continuity sensing
28                  IMU accelerometer data ready interrupt
29                  IMU gyroscope data ready interrupt
30                  On-board button
31                  NC [#f2]_
32                  NC [#f2]_
33                  NC [#f2]_
34                  External UART interface header RX
35                  External UART interface header TX
36                  NC [#f2]_
37                  NC [#f2]_
38                  NC [#f2]_
39                  NC [#f2]_
40                  NC [#f2]_
41                  Battery voltage sensing
==================  ======================================
 

.. rubric:: Footnotes

.. [#f1] The pin number refers to the pin number in software, 
   refer to the flight computer schematic for the electrical pinout.
.. [#f2] No electrical connection.
