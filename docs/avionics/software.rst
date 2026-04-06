Hardware abstraction layer (HAL)
================================
The hardware abstraction layer (HAL) of the flight software stack is intended
to separate the guidance and control code from the hardware. The HAL is
responsible for:

* initializing the sensors on the flight computer board,
* handling the calibration of the sensor stack,
* receiving the data from the sensors,
* transforming the data into the body frame coordinate system,
* performing sensor data filtering and fusion,
* providing guidance and control code with absolute position and orientation data,
* logging telemetry onto the onboard flash memory and SD card,
* providing an interface to control the servo motors to guidance and control,
* providing an interface for pyro channel control,
* and monitor the battery input voltage.

---------
Barometer
---------

------------
Magnetometer
------------

-------------------------
Inertial measurement unit
-------------------------

----------------
Servo controller
----------------

-----------------
Telemetry logging
-----------------

The telemetry logging API uses a buffered approach for writing data to flash
storage. Buffering is done using 2 buffers that are swapped on every buffer
flush.

Two interfaces for flushing a buffer are provided: the preferred asynchronous
interface that utilizes the DMA controller of the MCU to copy the data to the
internal flash memory, and the synchronous interface that manually performs the
copy at the cost of blocking the execution.

Writing is always performed on the currently active buffer as the inactive
buffer may currently be being read by the DMA. Additional attention is
necessary when performing a buffer swap to check whether the last DMA
controller is still copying the data from the last buffer.

Another interface for the copying of data from the internal flash memory to the
SD card is also provided. This interface is intended to be used at the end of
the flight to copy the entire flight telemetry log for further analysis by the
ground team.

---------------------------
Battery voltage measurement
---------------------------

---------------------------
Pyro channel control
---------------------------
