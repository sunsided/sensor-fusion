sensor data set #1
==================

This is the first data set. These measurements suffer from various problems:

- Sensors are not aligned to north. Because of this, when the reference system is interpreted as `magnetic north = +X`, rotations around the Y axis are not around reference Y which results in wobbly rotations.
- The MPU6050 accelerometer data is not giving the _gravity_ vector (`-Z`) but the _up_ vector (`Z`). This way, when gravity vector and magnetic field vector are 45° degree apart (e.g. magnetometer vector at `{1, 0, -1}`, gravity vector at `{-1, 0, -1}` and thus _up_ vector at `{-1, 0, 1}`), the cross product of accelerometer and magnetometer vectors is undefined, as they are perpendicular.
- The sign of the MPU6050's accelerometer and gyroscope's X axis is flipped

Even though all these glitches may be corrected in software, another data set was created to make testing easier.

### InvenSense MPU6050 ###

#### Speed ####

- measured at 100Hz

#### Type ####

- Accelerometer
- Gyroscope
- Temperature

### Honeywell HMC5883L ###

#### Speed ####

- measured at 75Hz

#### Type ####

- Magnetometer
