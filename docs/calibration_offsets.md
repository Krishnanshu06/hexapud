# HexaPud Servo Calibration & Offset Log
Date: 2026-10-20
Project: HexaPud 6-Leg Hexapod

## Calibration Offsets & Joint Direction Summary

| Leg Index | Name | Board (I2C) | Channels (C, F, T) | Direction [C, F, T] | Current Offsets [Coxa, Femur, Tibia] |
| :---: | :--- | :---: | :---: | :---: | :---: |
| **0** | Front-Left (White PLA) | Board 1 (`0x40`) | Ch 0, 1, 2 | `[ 1,  1,  1]` | `[ +5.0°, +10.0°,  -3.0°]` |
| **1** | Middle-Left (Black PLA)| Board 1 (`0x40`) | Ch 4, 5, 6 | `[ 1,  1,  1]` | `[  0.0°,  -2.0°, -21.0°]` |
| **2** | Back-Left (Black PLA)  | Board 1 (`0x40`) | Ch 8, 9, 10 | `[ 1,  1,  1]` | `[  0.0°,  +5.0°, -15.0°]` |
| **3** | Back-Right (Black PLA) | Board 2 (`0x41`) | Ch 0, 1, 2 | `[-1, -1, -1]` | `[  0.0°, -14.0°,  +3.0°]` |
| **4** | Middle-Right (Black PLA)| Board 2 (`0x41`) | Ch 4, 5, 6 | `[-1, -1, -1]` | `[ -8.0°,  -5.0°, +13.0°]` |
| **5** | Front-Right (Black PLA)| Board 2 (`0x41`) | Ch 8, 9, 10 | `[-1, -1, -1]` | `[ -7.0°,  -7.0°,  +4.0°]` |

---

### Hardware Notes & Joint Mapping
- **Coxa (Hip)**: 
  - Left Legs (0, 1, 2): Direction = `1`
  - Right Legs (3, 4, 5): Direction = `-1` (mirrored side)
- **Femur (Thigh)**:
  - Left Legs: Direction = `1`
  - Right Legs: Direction = `-1`
- **Tibia (Shin / Knee)**:
  - Left Legs: Direction = `1`
  - Right Legs: Direction = `-1`
- **Leg 0** is 3D printed in **White PLA** with ~90° tibia mount.
- **Legs 1 through 5** are 3D printed in **Black PLA** with ~75° tibia mount.
