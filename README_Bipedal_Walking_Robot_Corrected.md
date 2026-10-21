# Bipedal Walking Robot — Teensy 4.0, PCA9685 and MPU6050

A linkage-based biped robot reconstruction project documenting the known hardware, provisional servo mapping, staged commissioning firmware, CAD reconstruction and validation steps required before attempting walking.

> **Project status:** This repository contains a reconstructed CAD model and commissioning-stage firmware. The model is based on visual reference and uses assumed dimensions where measurements were unavailable. The firmware is not a validated autonomous walking controller.

## Contents

- [CAD assembly](CAD/biped_robot_reconstructed_full_assembly.step) — reconstructed full assembly in STEP format.
- [CAD notes and dimension assumptions](CAD/README_DIMENSIONS_AND_LIMITATIONS.md) — known limitations and dimensions requiring verification.
- [CAD generation script](CAD/generate_biped_cad.py) — source script used to generate the reconstructed model.
- [Teensy firmware](firmware/biped_robot_staged_firmware.ino) — staged servo and sensor commissioning firmware.
- [Technical report](docs/Biped_Robot_Rebuild_Technical_Report.docx) — reconstruction, electronics, safety and test planning.
- [Reference video](media/biped_robot_demo.mp4) — original visual reference, if included in this repository.

## Hardware

The current working hardware list is:

- Teensy 4.0 microcontroller
- PCA9685 16-channel PWM servo driver
- Six high-torque servos (nominal 15 kg·cm class; exact model to be confirmed)
- MPU6050 inertial measurement unit
- ADS1115 ADC module
- OV7670 camera module
- Linkage-based legs, pivot joints, feet and mounting plates

Some details are provisional. Confirm exact component models, dimensions, servo voltage ratings, wiring and mechanical mounting locations against the physical hardware before powering or manufacturing parts.

## Firmware setup

1. Install the Arduino IDE and the Teensy board support package.
2. Install the libraries required by the firmware:
   - Adafruit PWM Servo Driver Library
   - Adafruit MPU6050
   - Adafruit Unified Sensor
   - Adafruit BusIO
3. Open `firmware/biped_robot_staged_firmware.ino`.
4. Review the pin assignments, I²C addresses, servo channel mapping and pulse limits before connecting actuators.
5. Test with the robot safely supported and the servo power supply disabled until wiring and voltage compatibility have been checked.

The provisional servo channel map is a starting point only. Verify every channel and direction individually before using the assembly.

## Power and safety

- **Do not power servos from the Teensy 3.3 V pin.** Use a separate, appropriately rated servo supply.
- Verify the exact servo datasheet before choosing the servo supply voltage. Do not assume 8.4 V is safe for every servo.
- Use a common ground between the controller, driver and servo supply.
- Add appropriate overcurrent protection and a physical power cutoff.
- Teensy 4.0 GPIO is 3.3 V logic. Check I²C pull-up voltage levels; do not expose Teensy pins to 5 V.
- Keep the robot mechanically supported during commissioning and keep hands clear of linkages.

## CAD model limitations

The STEP assembly is a **visual reconstruction**, not the original source CAD. Link lengths, plate dimensions, pivot locations, servo mounts and clearances may be estimated. Validate the model against measurements of the real robot before using it for manufacturing, collision analysis or motion planning.

## Validation roadmap

1. Inspect CAD dimensions and interference against the physical assembly.
2. Verify power rails, grounds, I²C wiring and sensor communication with servos disconnected.
3. Test each servo channel individually at conservative pulse limits.
4. Confirm joint direction, mechanical travel and linkage interference with the robot supported.
5. Calibrate the IMU and validate orientation readings.
6. Develop and test gait control incrementally only after mechanical and electrical validation.

## Repository layout

```text
Bipedal-walking-robot/
├── README.md
├── CAD/
│   ├── biped_robot_reconstructed_full_assembly.step
│   ├── generate_biped_cad.py
│   └── README_DIMENSIONS_AND_LIMITATIONS.md
├── firmware/
│   └── biped_robot_staged_firmware.ino
├── docs/
│   └── Biped_Robot_Rebuild_Technical_Report.docx
└── media/
    ├── biped_robot_demo.mp4
    └── README.md
```

The paths above are the intended layout. If you have not moved the files into these folders yet, move them or update the links in this README to match their current locations.

## License

See [LICENSE](LICENSE).
