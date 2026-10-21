# Bipedal Walking Robot — Teensy 4.0 + PCA9685 + MPU6050

A linkage-based biped robot rebuild project. This repository documents the known hardware, a provisional servo channel map, staged commissioning firmware, and the steps required before attempting walking.

> **Status:** Commissioning scaffold, not a validated autonomous walking controller. Do not attempt unsupported walking until the geometry, servo limits, power budget, and balance strategy are verified.

## Hardware

| Component | Qty | Purpose | Status |
|---|---:|---|---|
| Teensy 4.0 | 1 | Main controller | Previously specified |
| PCA9685 16-channel PWM driver | 1 | Servo signal generation | Previously specified |
| 15 kg·cm servos | 6 | Leg/linkage actuation | Exact model unknown |
| MPU6050 | 1 | Body orientation estimate | Specified for rebuild |
| ADS1115 | 1 | Analog measurement | Previously specified |
| OV7670 camera | 1 | Future vision experiments | Previously specified |
| Servo supply | 1 | Servo power | Must be selected from datasheet |
| Logic supply | 1 | Teensy/sensors | Regulated supply required |

### Voltage warning

The intended servo rail was described as 7.4–8.4 V, but the exact servo model is unknown. **Do not connect 8.4 V until the servo datasheet explicitly permits it.** The stated 15 kg·cm rating is approximately 1.47 N·m at stall; stall torque is not a continuous-duty rating.

## Provisional servo map

This map is a placeholder and must be checked against the actual CAD assembly.

| PCA9685 channel | Provisional function |
|---:|---|
| 0 | Left hip pitch |
| 1 | Left knee/linkage |
| 2 | Left auxiliary/ankle linkage — verify |
| 3 | Right hip pitch |
| 4 | Right knee/linkage |
| 5 | Right auxiliary/ankle linkage — verify |

The exploded CAD video does not establish exact dimensions or all six physical servo locations. Rename these channels after tracing the real linkages.

## Firmware

Starter firmware: [`firmware/biped_robot_staged_firmware.ino`](firmware/biped_robot_staged_firmware.ino)

It initializes the PCA9685 and MPU6050, estimates pitch with a complementary filter, starts with servo outputs disarmed, and supports serial commands for safe commissioning. It does **not** implement walking or guarantee dynamic balance.

### Libraries

Install via Arduino Library Manager:
- Adafruit PWM Servo Driver Library
- Adafruit MPU6050
- Adafruit Unified Sensor
- Adafruit BusIO

Select **Teensy 4.0** in the Arduino IDE after installing the Teensy board package.

### Serial commands (115200 baud)

| Command | Action |
|---|---|
| `h` | Help |
| `s` | Status |
| `d` | Disarm servo outputs |
| `a` | Arm if IMU is healthy and pitch is near zero |
| `n` | Command stored neutral pulses |
| `j <ch> <deg>` | Command one channel, 0–5, logical 0–180° |
| `p <ch> <ticks>` | Set PCA9685 pulse ticks within configured limits |
| `c` | Recalibrate gyro bias while stationary |

The sketch's pulse limits and neutral values are generic placeholders. Calibrate each servo individually while the robot is supported.

## Wiring principles

- PCA9685 `VCC` is logic power; `V+` is the servo rail. Confirm the breakout board schematic.
- Use a dedicated servo supply; never power six servos from the Teensy's 3.3 V output.
- Join logic and servo grounds for signal reference, while routing high-current returns away from sensitive IMU wiring where practical.
- Teensy 4.0 uses 3.3 V logic. Avoid 5 V I²C pull-ups directly on Teensy pins.
- Confirm the Teensy I²C pins and I²C addresses for the PCA9685, MPU6050 and ADS1115.
- OV7670 is a parallel camera interface; module pinout and capture support depend on the specific breakout.

## Recover dimensions

Recover these from the original CAD or measure the assembled robot. Measure pivot-centre to pivot-centre, not bar outside length.

| Parameter | Symbol | Status |
|---|---|---|
| Overall height | H | Unknown |
| Upper plate width/depth | Wp, Dp | Unknown |
| Hip-to-knee pivot distance | L1 | Unknown |
| Knee-to-foot pivot distance | L2 | Unknown |
| Foot length/width | Lf, Wf | Unknown |
| Leg spacing | d | Unknown |
| Total mass / centre of mass | m / COM | Unknown |
| Servo horn radius | rh | Unknown |
| Joint limits | qmin, qmax | Unknown |

## Power budget

Calculate supply current using exact servo datasheets or measurements:

`I_peak_total = sum(I_peak for six servos) + I_logic + I_sensors`

Use a correctly rated fuse near the battery, suitable wiring/connectors, a physical power cutoff, and a regulated logic rail. Measure voltage sag and current during restrained tests. Do not size the supply from average current alone.

## Staged commissioning plan

1. Inspect wiring, polarity, linkage freedom and the physical cutoff with power disconnected.
2. Power logic only; verify serial output and stable IMU readings.
3. Calibrate gyro bias with the robot stationary; verify IMU axis/sign by hand tilt.
4. Support or tether the robot and test one servo at a time using small movements.
5. Record safe minimum, neutral and maximum pulse values for every servo.
6. Check binding, hard-stop collisions, heating, voltage sag and current.
7. Validate a supported static pose.
8. Test low-amplitude weight shifts while supported.
9. Add joint/foot feedback as needed, then develop gait trajectories.
10. Attempt walking only after all prior stages pass.

## Safety

- Keep hands clear of linkages while powered.
- Use a support frame/tether and keep a physical cutoff within reach.
- Stop if a servo stalls, heats rapidly, chatters, or a linkage binds.
- IMU-only feedback does not guarantee stable biped walking.
- Do not claim successful walking or measured balance performance without test logs.

## Known unknowns

Exact servo model, voltage range, peak current, speed; CAD dimensions; actual servo positions; battery capacity; calibrated pulse limits; encoders/foot sensors; measured mass and centre of mass; validated gait results.

## Files

- [`docs/Biped_Robot_Rebuild_Technical_Report.docx`](docs/Biped_Robot_Rebuild_Technical_Report.docx)
- [`docs/SETUP_NOTES.txt`](docs/SETUP_NOTES.txt)
- [`hardware/`](hardware/) — add wiring diagrams and dimensioned CAD exports
- [`media/`](media/) — add photos and test videos you have permission to publish

## License

An MIT license is included as a suggested default. Keep it only if it matches your intended sharing terms.
