# Sensor data set #2

In all measurements the sensors were aligned to gravity and north as much as possible.

## Coordinate System Convention

Right-handed coordinate system with:

- **X:** forward direction
- **Y:** left direction
- **Z:** up direction

### Sensor orientation

- **Accelerometer:**
  - **X:** forward (same as reference)
  - **Y:** left (same as reference)
  - **Z:** up (same as reference)
- **Magnetometer:**
  - **X:** ...
  - **Y:** ...
  - **Z:** up (same as reference)
- **Gyroscope:**
  - **X:** positive is clockwise, **negative is counter-clockwise** - flipped sign
  - **Y:** **positive is counter-clockwise**, negative is clockwise
  - **Z:** **positive is counter-clockwise**, negative is clockwise

### Example readings

#### X forward, Y left, Z up

Normal orientation. This should give a negative accelerometer reading on the Z axis,
as it points upwards.

See [`unmoved-with-x-pointing-forward-10min`](unmoved-with-x-pointing-forward-10min/) or [`nmoved-with-x-pointing-forward`](nmoved-with-x-pointing-forward/).

| Sensor |  X   |  Y   |  Z   |
|--------|------|------|------|
| Accel  |  0   |  0   | -1   |
| Gyro   |  5   |  0   |  1   |
| Mag    |  0.2 | -0.1 | -0.3 |

#### X up, Y left, Z forward

Rotated clockwise around y, "bringing the nose up". This should give a negative accelerometer
reading on the X axis, as it points upwards.

See [`unmoved-with-z-pointing-front-and-x-up`](unmoved-with-z-pointing-front-and-x-up/).

| Sensor |  X   | Y    | Z    |
|--------|------|------|------|
| Accel  | -1   |  0   |  0   |
| Gyro   |  5   |  0   |  1   |
| Mag    | -0.5 | -0.1 |  0.3 |

#### X left, Y down, Z forward

Rotated 90 degree to the left (CCW around Z), then 180 degree around the left "on the head".
This should give a positive accelerometer reading on the Y axis, as it points downwards.

See [`unmoved-with-z-pointing-front-and-y-down`](unmoved-with-z-pointing-front-and-y-down/).

| Sensor |  X   | Y    | Z    |
|--------|------|------|------|
| Accel  |  0   |  1   |  0   |
| Gyro   |  5   |  0   |  1   |
| Mag    |  0   | 0.4  | 0.3  |

## Sensors

### InvenSense MPU6050

#### Speed

- measured at 100Hz

#### Type

- Accelerometer
- Gyroscope
- Temperature

### Honeywell HMC5883L

#### Speed

- measured at 75Hz

#### Type

- Magnetometer
