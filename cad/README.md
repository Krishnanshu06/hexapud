# 3D Print Files & CAD Models

This directory contains all printable files for building the HexaPud chassis and 18-DOF legs.

## Chassis Structure (`cad/body/`)

All main body files are provided in `.3mf` format (compatible with Bambu Studio, PrusaSlicer, Cura, OrcaSlicer, etc.):

| File | Quantity | Notes |
| :--- | :---: | :--- |
| `body_main_bottom.3mf` | 1 | Lower chassis plate with electronics mounting bosses and leg pivot anchors |
| `body_main_top.3mf` | 1 | Upper chassis plate |
| `body_lid.3mf` | 1 | Electronics protection cover lid |
| `body_spacer_set.3mf` | 1 set | Standoff spacers between the top and bottom chassis plates |

---

## Leg Segments & Brackets (`cad/legs/`)

A full hexapod requires **6 legs total**:
- **3 Right Legs** (Legs 3, 4, 5) printed using files in `right_leg/`
- **3 Left Legs** (Legs 0, 1, 2) printed using mirrored files in `left_leg_mirrored/`
- **Symmetrical Parts** (all 6 legs) printed using files in `common/`

### 1. Common Parts (`cad/legs/common/`)
*These parts are symmetrical and identical for both left and right legs.*

| File | Quantity | Notes |
| :--- | :---: | :--- |
| `tibia_hollowed.stl` | 6 | Redesigned monocoque tibia. Shelled single-piece design with hollowed-out interior to minimize print material and lighten knee servo load |
| `bushing.stl` | 12 | Low-friction hinge bushings for joint pivot pins |

### 2. Right Leg Parts (`cad/legs/right_leg/`)
*Print 3 sets of these parts for the right side (Legs 3, 4, 5).*

| File | Quantity | Notes |
| :--- | :---: | :--- |
| `coxa.stl` | 3 | Horizontal yaw link attaching leg to chassis |
| `femurSlimPart.stl` | 3 | Slim half of the two-part femur thigh assembly |
| `femurThicPart.stl` | 3 | Thick half of the two-part femur thigh assembly |
| `servo_mount_trochanter.stl` | 3 | Shoulder pitch servo bracket (modified for MG996R rear PCB clearance) |
| `servo_mount_tibia.stl` | 3 | Knee pitch servo bracket |

### 3. Left Leg Mirrored Parts (`cad/legs/left_leg_mirrored/`)
*Print 3 sets of these parts for the left side (Legs 0, 1, 2).*

| File | Quantity | Notes |
| :--- | :---: | :--- |
| `leg_set_mirrored(set of coxa, femurThic and slim).stl` | 3 | Combined plate containing all three mirrored leg segments: the mirrored coxa, femurThicPart, and femurSlimPart |
| `servo_mount_trochanter_mirrored.stl` | 3 | Mirrored shoulder pitch servo bracket |
| `servo_mount_on_tibia_mirrored.stl` | 3 | Mirrored knee pitch servo bracket |

---

## Recommended Print Settings

- **Filament**: PLA or PETG
- **Infill**: 35% Gyroid (provides balanced torsional rigidity across all axes)
- **Perimeters / Wall Loops**: 3 to 4 walls for strong screw hole threads and heat-set inserts
- **Layer Height**: 0.20 mm standard
- **Heat-Set Inserts**: M5 brass inserts (OD 7mm, length 5mm) pressed into chassis mounting bosses
