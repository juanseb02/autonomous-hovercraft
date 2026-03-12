# Autonomous Maze-Navigating Hovercraft

**Team #3 · Fall 2025**

An autonomous hovercraft built on an ATmega328P that navigates a physical maze using IR-based wall detection, gyroscope-assisted straight-line driving, and a greedy intersection solver.

---

## Hardware

| Component | Role |
|---|---|
| ATmega328P (16 MHz) | Main microcontroller |
| MPU-6050 (I2C) | Gyroscope for yaw tracking |
| Sharp GP2Y IR sensor | Front wall / obstacle detection |
| HC-SR04 ultrasonic | Upward-facing bar detection (finish line) |
| Hobby servo | Thrust vectoring (steering) |
| Brushless lift fan | Keeps hovercraft airborne |
| Brushless thrust fan | Forward propulsion |

---

## Software Architecture

```
main.c      — Entry point, 1 kHz systick ISR, software PWM for fans
config.h    — All pin definitions, thresholds, and tuning constants
imu.c/h     — TWI (I2C) driver, MPU-6050 init, gyro calibration, yaw integration
sensors.c/h — ADC driver, IR distance sensor, HC-SR04 ultrasonic driver
pid.c/h     — PID yaw correction controller feeding the steering servo
motion.c/h  — Servo/fan control, turn_by_yaw, straight drive, intersection logic
```

---

## Navigation Algorithm

1. **Drive straight** — PID loop corrects heading drift using integrated gyro yaw.
2. **Wall detected** — IR sensor triggers a stop when within `OBSTACLE_THRESHOLD_CM`.
3. **Scan intersection** — Servo sweeps left and right; IR measures open space in each direction.
4. **Turn decision** — Greedy: turn toward the more open side. U-turn on dead end.
5. **Finish detection** — Upward HC-SR04 confirms the overhead bar over multiple readings; hovercraft stops and blinks LED.

---

## Building & Flashing

Requires `avr-gcc` and `avrdude`.

```bash
make          # compile
make flash    # upload via Arduino bootloader on /dev/ttyUSB0
make clean    # remove build artifacts
```

Adjust `PORT` in the Makefile if your programmer is on a different serial port.

---

## Key Tuning Parameters (`config.h`)

| Constant | Default | Description |
|---|---|---|
| `YAW_KP/KI/KD` | 3.0 / 0.1 / 0.8 | PID gains for heading correction |
| `OBSTACLE_THRESHOLD_CM` | 60 | Stop distance from front wall |
| `THRUST_CRUISE_DUTY` | 80% | Fan duty during straight driving |
| `UPBAR_THRESHOLD_CM` | 25 | Max distance to detect finish bar |
