# icm20948_driver change log

## v X.Y.Z
* Removed unnecessary cortex-m dependency.
* Made defmt a feature rather than required.
* Added functions to specify the sample rate dividers directly.
* Added attributes including Debug, Copy, and Clone to IcmError.
* More thorough testing on data rates and interrupts. The gyro data rate determines the update rate for the raw data interrupt if the gyro is enabled. 
* Added interrupt examples.
* Fixed enable and disable bug for accelerometer and gyroscope (for SPI).
* Made the bus error more specific (TODO: Test this).
* Added an unstable reset function.

## v 0.1.0
* Read accelerometer and gyroscope.
* Enable the raw data interrupt.
* Change the data rates.
* Adjust the low pass filter.
* Who am I.
* I2C and SPI.
* Set accelerometer and gyroscope sensitivity.
* Enable and disable features.