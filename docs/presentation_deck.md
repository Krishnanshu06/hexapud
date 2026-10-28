# HexaPud: 4-Slide Presentation Deck

---

## Slide 1: Locomotion & Gait Dynamics (The Alternating Tripod Gait)

### Slide Header
- **Title**: Locomotion & Gait Dynamics: The Alternating Tripod Gait
- **Subtitle**: Bio-inspired Static Stability for Omnidirectional Terrain Traversal

### Key Content Points (On Slide)
- **Static Stability & The Support Polygon**:
  - A walking robot is statically stable when its Center of Mass (CoM) stays inside the polygon formed by grounded feet.
  - By keeping 3 feet planted at all times, the robot forms an immovable **Support Triangle**.
- **Alternating Tripod Teams**:
  - **Tripod A**: Leg 0 (Front-Left), Leg 2 (Back-Left), Leg 4 (Middle-Right).
  - **Tripod B**: Leg 1 (Middle-Left), Leg 3 (Back-Right), Leg 5 (Front-Right).
  - Phase offset of $180^\circ$ ($\Delta\phi = 0.5$): While Team A swings in mid-air, Team B supports and drives the chassis forward.
- **The 3D "D-Loop" Trajectory**:
  - **Swing Phase (In the Air)**: Follows a sinusoidal arch ($Z = Z_{\text{ground}} + H_{\text{lift}}\sin(\pi t)$) to clear ground obstacles.
  - **Stance Phase (On the Floor)**: Foot stays flat on the floor ($Z = -140\text{mm}$) pushing backward to propel the robot forward.

### Visuals for Slide 1
- **Animated GIF**: [d_loop_animation.gif](assets/d_loop_animation.gif) *(Smooth looping 50Hz foot step cycle)*
- **High-Res Diagram**: [d_loop_trajectory.png](assets/d_loop_trajectory.png) *(300 DPI diagram with waypoints, dimensions, and formulas)*
- **Top-Down Walking GIF**: [tripod_gait_animation.gif](assets/tripod_gait_animation.gif) *(Looping top-down hexapod showing alternating Tripod A and B support triangles)*

```
+-------------------------------------------------------------+
|  [INSERT: d_loop_animation.gif OR tripod_gait_animation.gif] |
|  * Live looping animation showing Swing vs. Stance phase    |
+-------------------------------------------------------------+
|  [INSERT: d_loop_trajectory.png]                            |
|  * Labeled trajectory curve with Liftoff, Apex, and Landing  |
+-------------------------------------------------------------+
```

### Speaker Notes (What to Say - 60 to 90 seconds)
> *"Good morning/afternoon everyone. Today I'm presenting HexaPud, an 18-servo bio-inspired robot.*
>
> *Slide 1 covers how the robot actually walks. Unlike humans who are dynamically balanced and constantly falling forward, a hexapod uses **static stability**. At any instant during walking, 3 of its 6 legs are firmly planted on the ground, creating a triangular base of support with the center of gravity safely in the middle.*
>
> *We divide the legs into two alternating teams: Tripod A and Tripod B. While Tripod A lifts into the air along a smooth sinusoidal arch to take a step forward, Tripod B stays flat on the floor pushing the body forward. When Tripod A touches down, the roles instantly reverse. Because every step forms a closed 3D loop shaped like the letter 'D', the motion is continuous, balanced, and completely shock-free."*

---

## Slide 2: Geometric Inverse Kinematics (IK)

### Slide Header
- **Title**: Inverse Kinematics (IK): From 3D Space to Servo Angles
- **Subtitle**: Analytical Trigonometric Solution for 3-DOF Robotic Limbs

### Key Content Points (On Slide)
- **The Core Problem**:
  - Humans and path planners think in **Cartesian Coordinates $(X, Y, Z)$ in millimeters**.
  - Servos only understand **Rotational Angles $(\theta_{\text{coxa}}, \theta_{\text{femur}}, \theta_{\text{tibia}})$ from $0^\circ$ to $180^\circ$**.
- **The 3-DOF Link Breakdown**:
  - **Coxa ($L_1 = 51\text{mm}$)**: Horizontal yaw (swings forward/backward).
  - **Femur ($L_2 = 80\text{mm}$)**: Vertical thigh pitch (lifts body up/down).
  - **Tibia ($L_3 = 128\text{mm}$)**: Knee pitch (sets foot extension and reach).
- **The Mathematical Solution**:
  1. **Hip Yaw**: $\theta_{\text{coxa}} = \text{atan2}(Y, X)$
  2. **Planar Reach**: $d_{\text{horiz}} = \sqrt{X^2 + Y^2} - L_1 \quad \rightarrow \quad D = \sqrt{d^2 + Z^2}$
  3. **Law of Cosines (Knee)**: $\cos(\gamma) = \frac{L_2^2 + L_3^2 - D^2}{2 L_2 L_3} \quad \rightarrow \quad \theta_{\text{tibia}} = \arccos(\cos\gamma)$
  4. **Femur Elevation**: $\theta_{\text{femur}} = \text{atan2}(-Z, d) - \arccos\left(\frac{L_2^2 + D^2 - L_3^2}{2 L_2 D}\right)$

### Visuals for Slide 2
- **High-Res Diagram**: [inverse_kinematics_diagram.png](assets/inverse_kinematics_diagram.png) *(300 DPI 3-DOF leg geometric derivation with Law of Cosines & formula box)*

```
+-------------------------------------------------------------+
|  [INSERT: inverse_kinematics_diagram.png]                   |
|  * Showing Coxa L1, Femur L2, Tibia L3, reach D, and        |
|    the complete analytical equations in a dark theme        |
+-------------------------------------------------------------+
```

### Speaker Notes (What to Say - 60 to 90 seconds)
> *"On Slide 2, we answer the mathematical question: how do we calculate the exact servo angles?*
>
> *If you tell a servo to move to an arbitrary angle, the leg moves randomly. What we actually want is to place the foot at an exact $(X, Y, Z)$ point in space—for example, $130\text{mm}$ outward and $140\text{mm}$ below the shoulder. This is called **Inverse Kinematics**.*
>
> *Each leg has 3 joints: Coxa at the hip, Femur at the thigh, and Tibia at the shin. We solve this analytically in two quick steps:*
> *First, we look top-down: an `atan2` on $X$ and $Y$ immediately gives the horizontal Coxa angle.*
> *Second, we project the remaining two links into a vertical 2D plane. By connecting the hip to the foot tip, we form a triangle. Using the high-school Law of Cosines, we calculate the internal angles $\beta$ and $\gamma$.*
> *This produces the exact angles for all 3 servos in less than $0.1$ milliseconds, allowing us to recalculate all 18 servos 50 times every second."*

---

## Slide 3: Real-Time Software & Vector Pipeline

### Slide Header
- **Title**: Real-Time Motion Engine & Coordinate Pipeline
- **Subtitle**: Transforming High-Level Pilot Commands into Synchronized 50Hz PWM

### Key Content Points (On Slide)
- **Global Body Frame $\rightarrow$ Local Leg Frames**:
  - The pilot gives world-level commands (e.g., *Forward*, *Diagonal-Left*).
  - A **2D Rotation Matrix** rotates the world heading vector into each leg's unique mounting angle ($\pm 30^\circ, \pm 90^\circ, \pm 150^\circ$):
    $$\begin{bmatrix} X_{\text{local}} \\ Y_{\text{local}} \end{bmatrix} = \begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix} \begin{bmatrix} V_x \\ V_y \end{bmatrix}$$
- **The Stride Progress Slider ($P \in [-1.0, +1.0]$)**:
  - $P$ is a dimensionless progress factor.
  - Multiplied by the half-stride amplitude to produce smooth back-and-forth oscillations around the neutral standing point ($X = 130\text{mm}$).
- **Non-Blocking Architecture (`select.poll`)**:
  - ESP32 MicroPython control loop runs at a rigid **50Hz (every 20ms)**.
  - Non-blocking network polling (`poll(0)`) checks for user commands in $<0.05\text{ms}$ without stalling the gait execution.

### Slide Architecture Overview
- **Pipeline Flow**: Pilot HTTP Request $\rightarrow$ Wi-Fi SoftAP $\rightarrow$ Non-blocking `select.poll(0)` $\rightarrow$ 2D Heading Rotation $\rightarrow$ 50Hz D-Loop Clock $\rightarrow$ 3-DOF Analytical IK $\rightarrow$ Servo Calibration Offsets $\rightarrow$ Dual PCA9685 PWM Drivers (400kHz I2C).

### Speaker Notes (What to Say - 60 to 90 seconds)
> *"Slide 3 breaks down the software pipeline running inside the ESP32.*
>
> *The key challenge is coordination. When the pilot pushes the joystick forward, each of the 6 legs faces in a completely different direction. Leg 0 points forward-left, while Leg 1 points sideways. The software takes the global forward vector and multiplies it by a 2D rotation matrix for each leg's mount angle. That means Leg 1 knows it must swing sideways relative to its shoulder to push the robot forward in world space.*
>
> *Next, our motion engine uses a dimensionless slider called **$P$**, which sweeps smoothly from $-1.0$ to $+1.0$ using S-curve interpolation. This guarantees zero mechanical jerk when legs lift off or touch down.*
>
> *Finally, all network commands—including the Emergency Stop—are handled using non-blocking socket polling. This eliminates operating system tick delays, ensuring the 18 servos receive their PWM signals every 20 milliseconds like clockwork."*

---

## Slide 4: Dual-Mode Architecture & Interactive System

### Slide Header
- **Title**: Dual-Mode Capabilities & Embedded System Design
- **Subtitle**: 6-Leg Hexapod vs. 4-Leg Quadruped Morphing on ESP32

### Key Content Points (On Slide)
- **Dual-Mode Morphing**:
  - **Hexapod Mode (6-Leg)**: Full 6-leg alternating tripod gait for high speed, maximum payload, and rough terrain traversal.
  - **Quadruped Mode (4-Leg)**: Middle legs tuck into an elevated "mantis" pose ($142^\circ$ femur); the 4 corner legs walk using a wide diagonal crawl gait.
- **Hardware Architecture**:
  - **MCU**: ESP32 Dual-Core running MicroPython.
  - **Actuation**: 18x MG996R Metal-Gear Servos powered by dual PCA9685 I2C PWM controllers at 400kHz.
  - **Network**: Self-hosted Wi-Fi SoftAP (`192.168.4.1`) serving an interactive web cockpit.
- **Control Features**:
  - 8-direction snap-to-latch virtual joystick with duration mapping.
  - Real-time chassis height elevation slider ($\pm 50\text{mm}$).
  - Live locomotion speed profiles (Normal, Fast, Turbo) and instant multi-channel Emergency Stop.

### Visuals for Slide 4
- **Physical Hardware Photo**: [complete_assembly.jpg](media/complete_assembly.jpg) *(Full assembled hexapod profile)*
- **Internal Component Placement**: [component_positions_base.jpg](media/component_positions_base.jpg) *(Dual PCA9685, ESP32, and dual-buck converter arrangement)*
- **Hardware Walking GIF**: [robot_walk.gif](media/robot_walk.gif) *(Physical robot walking test)*

```
+-------------------------------------------------------------+
|  [INSERT: robot_walk.gif]                                   |
|  * Video capture of the physical robot walking on floor     |
+-------------------------------------------------------------+
|  [INSERT: complete_assembly.jpg]                            |
|  * Complete hardware robot assembly showing lightweight legs|
+-------------------------------------------------------------+
```

### Speaker Notes (What to Say - 60 to 90 seconds)
> *"Finally, Slide 4 shows the complete robot system and its unique capabilities.*
>
> *The entire robot is self-contained. The ESP32 hosts its own Wi-Fi Access Point, serving an interactive web controller to any smartphone or laptop without needing an internet connection. It features an 8-way latching joystick, variable speed selection, and chassis elevation controls.*
>
> *One of the most exciting features we engineered is **Dual-Mode Morphing**. With a single tap on the UI, the robot can switch between a standard 6-legged hexapod and a 4-legged quadruped. In quadruped mode, the two middle legs fold up into the air like mantis claws, and the software dynamically switches from a tripod gait to a diagonal crawl gait.*
>
> *Thank you, and I'd be happy to answer any questions or demonstrate the robot live!"*

---

## Quick Presentation Checklist
- **Duration**: ~4 to 5 minutes total (~1 minute per slide).
- **Key buzzwords to emphasize**: *Static Stability, Support Triangle, Inverse Kinematics, Law of Cosines, Parametric Trajectory, Non-Blocking Event Loop, Dual-Mode Morphing*.
- **Live Demo Cue**: Transition to a live walk burst or chassis twist after Slide 4!
