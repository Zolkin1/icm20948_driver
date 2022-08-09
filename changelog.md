# icm20948_driver change log

## v X.Y.Z
* Removed unnecessary cortex-m dependency.
* Made defmt a feature rather than required.
* More thorough testing on data rates and interrupts. The gyro data rate determines the update rate for the raw data interrupt if the gyro is enabled. 
* Added interrupt examples.

## v 0.1.0
* Read accelerometer and gyroscope.
* Enable the raw data interrupt.
* Change the data rates.
* Adjust the low pass filter.
* Who am I.
* I2C and SPI.
* Set accelerometer and gyroscope sensitivity.
* Enable and disable features.