self-balancing-robot-psoc4
==========================

Self Balancing robot using the Cypress PSoC4.

Usefuleness of this repo:
1. C functions to interface to MPU9150 via I2C (MPU9150.h, MPU9150.c)
  - Replace I2C functions with your application specific functions, because this is dependent on the PSoC4 libraries.
2. main.c: Contains C code for implementing:
  - Complementary filter algorithm for combining gyro and accelerometer data to form estimated roll/pitch
  - PID algorithm
