##Teensy GPS Shield Library for GPS Data Logging

###Introduction
Teensy GPS Shield is a "work in progress" and is currently not ready for out of the box operation.  It started out as a GPS data logger project designed around the Teensy 3.1 microcontroller.  Recently, the project has expanded to include a few more features which include:

1.	MicroSD for Data Logging 
2.	GPS Receiver with 50Hz Update Rate
3.	9DOF Sensor
4.	CAN Bus Transceiver

These 4 features provide the essential data needed in many real world applications including RC aircraft, RC cars, UAVs, automotive racing, motor-cross and various other hobby and professional fields.  In addition, having sufficient flexibility built into the data logging application would provide a system that could be useful for scientific studies such as measuring sea ice drift and at the same time being cost effective for the general hobbyist to use in RC vehicles.

![TeensyGPS](http://www.smokingresistor.com/wp-content/uploads/2015/09/IMG_0001.jpg)

###GPS Receiver
The GPS receiver is based around the SkyTraq [Venus838FLPX](http://www.smokingresistor.com/wp-content/uploads/2014/06/Venus838FLPx_DS_v4.pdf) receiver chip which is capable of positon update rates of 50Hz.  Configuration of the GPS receiver is achieved through serial commands that will be transmitted from the Teensy microcontroller via serial pins 9 & 10.

###9DOF Sensor
The 9DOF sensor based around the ST Microelectronics [LSM9DS0TR](http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00087365.pdf) chip.  This chip has sensor data available in all 3 axes for acceleration, angular rates and magnetic fields.  The serial interface to this device is via an I2C bus on Teensy pins 16 & 17.  The sensor data from this chip is brought through several Teensy pins and also can be logged to the MicroSD card.

###MicroSD Card
The data from the GPS Receiver and 9DOF sensor get recorded to an onboard MicroSD card. Choosing which parameters will get recorded will be configurable via a config.txt file that resides on the MicroSD.  When the device is powered on, the Teensy will read the config.txt file and start recording data based on those parameters.

###CAN Bus Transceiver
The Teensy uses the [FlexCAN](https://github.com/teachop/FlexCAN_Library) library to communicate to a CAN bus transceiver chip which is part number [SN65HVD232DR](http://www.ti.com/lit/ds/symlink/sn65hvd230.pdf) by Texas Instruments.  The CAN bus interface is a DSUB 9 pin connector which is standard in most automotive CAN bus applications.

###Optional Kalman Filtering
Due to the inaccuracies of commercial GPS position data, the 50 Hz max data rate from the GPS receiver tends to be very noisy.  To overcome this noisy GPS data, a Kalman filter is being added.


