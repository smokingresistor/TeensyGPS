##Teensy GPS Shield TODO

###Checklist In Order of Importance
- [x] **Add Documentation** that includes the goals and direction of this project.
- [x] **Fix Kalman Filter** - The Kalman filter library has been written, but is not working correctly.  Raw latitude, longitude and velocity values match the filtered latitude, longitude and velocity filters exactly indicating the filter is either not working correctly or is not being called correctly.
- [x] **Merge 9DOF Sensor Library** - This project uses the ST Microelectronics [LSM9DS0TR](http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/DM00087365.pdf) chip.  An Arduino library for this chip already exists and can be found [here](https://github.com/adafruit/Adafruit_LSM9DS0_Library).
- [x] **Write Data Logger Code** - Currently, this project logs basic lat, long, velocity data to a MicroSD card.  A more versatile logging system is desired which is defined in the SoftwareDescription.docx file.  In essence, a configuration file that defines logging parameters will be stored on the MicroSD card.  This file gets loaded at startup and the system only logs the data as defined by the configuration file.
