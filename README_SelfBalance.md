# Self-Balancing Robot — State Feedback Control (Inverted Pendulum)

<p align="center">
  <img src="4- Multimedia/SelfBalanceRobot.png" width="70%" alt="Self-Balancing Robot — Paco"/>
</p>

> *"Behold, traveler of the edge, the dance of an upright body against the tyranny of gravity!"*

A two-wheeled self-balancing robot (also known as an **inverted pendulum on wheels**), designed and implemented as a Final Project for the **Control Systems Design** course at the Faculty of Engineering, Universidad Nacional de Asunción — CITEC-Luque, Paraguay.

The project covers the full engineering pipeline: **physical modelling → mathematical derivation → optimal control design (LQR) → MATLAB simulation → Arduino embedded implementation**.

---

## Table of Contents

1. [Project Overview](#project-overview)
2. [Team](#team)
3. [System Description & Physical Model](#system-description--physical-model)
4. [Mathematical Modelling (Lagrange-Euler)](#mathematical-modelling-lagrange-euler)
5. [State-Space Representation](#state-space-representation)
6. [Controller Design (Discrete LQR)](#controller-design-discrete-lqr)
7. [Hardware & Sensors](#hardware--sensors)
8. [Embedded Implementation (Arduino)](#embedded-implementation-arduino)
9. [Repository Structure](#repository-structure)
10. [Software Architecture](#software-architecture)
11. [Results](#results)
12. [References](#references)

---

## Project Overview

The **Self-Balancing Robot** (nicknamed **"Paco"**) is a two-wheeled mobile platform whose body sits vertically above its wheel axle — an inherently unstable configuration analogous to an inverted pendulum. Without active control, the robot falls immediately.

The project implements a **full state-feedback controller** (LQR) running on an **Arduino Uno** at 10 ms sampling rate. Four state variables are estimated in real-time from on-board sensors and fed back to compute the PWM motor command that keeps the robot upright.

| Property | Value |
|---|---|
| Control Strategy | Discrete LQR (Linear Quadratic Regulator) |
| Sampling Time | 10 ms (100 Hz) |
| Microcontroller | Arduino Uno |
| Motor Driver | L298N (dual H-bridge) |
| IMU Sensor | MPU-6050 (gyroscope + accelerometer) |
| Wheel Sensors | Two optical tachometers (interrupt-driven) |
| Programming Languages | MATLAB (design), C++ / Arduino (embedded) |

---

## Team

| Role | Name |
|---|---|
| Student | Lucas Pin |
| Student | Matteo Martinez |
| Student | Franco de Vargas |
| Advisor | Dr. Ing. Leonardo Comparatore |
| Advisor | MSc. Ing. Christian Medina |
| Advisor | MSc. Ing. Paola Maidana |

*Faculty of Mechatronic Engineering — Universidad Nacional de Asunción, CITEC-Luque, Paraguay.*

---

## System Description & Physical Model

The robot body is approximated as a **coupled rigid-body system**:

- **Two hollow cylinders** (the wheels) that roll without slipping, sharing the same instantaneous linear displacement.
- **Three rigid rectangular prisms** (prismatic bodies) representing the structural layers of the robot:
  - **Prism 1** — Battery + DC Motors + Tachometer discs (lowest layer, closest to the wheel axle)
  - **Prism 2** — Arduino Uno + L298N motor driver (mid layer)
  - **Prism 3** — MPU-6050 IMU sensor (top layer)

Each prism `i` has mass `m_i` and its centre of mass is at perpendicular distance `h_i` from the wheel axle (which serves as the reference `y = 0`).

### Physical Parameters

| Component | Mass (kg) | Distance from axle h (m) |
|---|---|---|
| Battery | 0.125 | — |
| Motors × 2 | 0.034 each | — |
| Tachometers × 2 | 0.040 each | — |
| **Prism 1 total** | **0.283** | **0.001** |
| Arduino Uno | 0.025 | — |
| L298N Driver | 0.025 | — |
| **Prism 2 total** | **0.050** | **0.052** |
| MPU-6050 | 0.002 | — |
| **Prism 3 total** | **0.002** | **0.100** |
| Wheels × 2 | 0.029 each | r = 0.034 m, r_int = 0.025 m |

The three assumptions that reduce the dynamics to a **2D problem**:
1. The wheels roll without slipping, and both share the same linear displacement at all times.
2. The wheel axle and the prism assembly are rigidly coupled.
3. The mass distribution is symmetric so the centre of mass of each cylinder lies in the plane of symmetry.

---

## Mathematical Modelling (Lagrange-Euler)

The **Lagrange-Euler** formulation is applied using two generalised coordinates:

- `q1 = x` — horizontal position of the wheel axle
- `q2 = θ` — pendulum angle measured from the vertical

### Kinetic Energy

**Wheels** (rolling without slipping):

```
T_wheels = (3/4) * m_r * x_dot^2
```

**Prisms** (translational + rotational contribution through coupling):

```
T_prisms = (1/2) * (m_T * x_dot^2 + 2*M_h * x_dot*theta_dot*cos(θ) + I_h * theta_dot^2)
```

where:
- `m_T = Σ m_i`
- `M_h = Σ m_i * h_i`
- `I_h = Σ m_i * h_i²`
- `M = m_T + (3/2)*m_r`

### Potential Energy

```
V = Σ m_i * h_i * g * cos(θ) = C1 * cos(θ),  with C1 = g * M_h
```

### Equations of Motion (Nonlinear)

```
M * x_ddot + M_h*cos(θ) * θ_ddot = F - b*x_dot + M_h*sin(θ)*θ_dot^2
I_h * θ_ddot + M_h*cos(θ) * x_ddot = C1*sin(θ) - c*θ_dot
```

where `b` is linear friction and `c` is angular friction (neglected: `c = 0`).

### Linearisation around θ = 0

Using small-angle approximations (`cos θ ≈ 1`, `sin θ ≈ θ`, `θ_dot² ≈ 0`):

```
M * x_ddot + M_h * θ_ddot = F - b*x_dot
I_h * θ_ddot + M_h * x_ddot = C1*θ - c*θ_dot
```

---

## State-Space Representation

**State vector:** `X = [x, x_dot, θ, θ_dot]ᵀ`

Solving the coupled linear system for `x_ddot` and `θ_ddot` (with `Δ = M*I_h - M_h²`):

```
x_ddot  = -(I_h*b/Δ)*x_dot + (M_h*C1/Δ)*θ - (M_h*c/Δ)*θ_dot + (I_h/Δ)*u
θ_ddot  =  (M_h*b/Δ)*x_dot + (M*C1/Δ)*θ  - (M*c/Δ)*θ_dot  - (M_h/Δ)*u
```

**Numerical state matrix A:**
```
A =
[ 0       1.0000    0        0      ]
[ 0      -0.2847    1.6946   0      ]
[ 0       0         0        1.0000 ]
[ 0       5.6270  227.1964   0      ]
```

**Numerical input matrix B:**
```
B = [0, 2.8469, 0, -56.2702]ᵀ
```

**Output matrix C** (linear position only — the sole observable combination):
```
C = [1, 0, 0, 0],  D = 0
```

### Discretisation — ZOH, Ts = 10 ms

ZOH was selected because it faithfully represents how control signals remain constant between updates in real digital systems.

**Discrete state matrix G:**
```
G =
[ 1.0000   0.0100   0.0001   0.0000 ]
[ 0        0.9972   0.0170   0.0001 ]
[ 0        0.0003   1.0114   0.0100 ]
[ 0        0.0564   2.2811   1.0114 ]
```

**Discrete input matrix H:**
```
H = [0.0001, 0.0284, -0.0028, -0.5640]ᵀ
```

The discrete system is **completely controllable** and **observable** when the linear position `x` is used as output.

---

## Controller Design (Discrete LQR)

### Design Choice: Direct State Feedback (No Observer)

Although an observer-based servo system was considered, it was discarded because:
- Linear position `x` cannot be measured directly and requires numerical integration of tachometer readings, introducing accumulated error.
- Observer quality heavily depends on model accuracy.
- A simpler **direct state-feedback** law was chosen instead.

### LQR Gain Computation

The feedback matrix `K` minimises the infinite-horizon quadratic cost:

```
J = Σ (X[k]ᵀ Q X[k] + u[k]ᵀ R u[k])
```

**Tuning matrices (high penalty on position and angle):**
```matlab
Q = diag([200, 10, 200, 20])
R = 1
K = dlqr(G, H, Q, R)
```

**Resulting gain vector:**
```
K = [-4.7152,  -3.3278,  -17.0580,  -1.8229]
```

### Closed-Loop Difference Equation

```
X[k+1] = G*X[k] + H*u[k]
u[k]   = -K*X[k]
```

---

## Hardware & Sensors

| Component | Role |
|---|---|
| Arduino Uno | Main microcontroller |
| MPU-6050 (I²C @ 400 kHz) | Gyroscope + Accelerometer — pitch angle and rate estimation |
| 2× Optical Tachometers | Interrupt-driven wheel speed (20 pulses/rev) |
| L298N Dual H-Bridge | Motor driver — accepts PWM commands |
| 2× DC Motors | Wheel actuation |
| LiPo Battery (125 g) | Power supply |

### State Variable Estimation Pipeline

| State | Estimation Method |
|---|---|
| `θ` (pitch angle) | MPU-6050 via 1-D Kalman filter (gyro + accelerometer fusion) |
| `θ_dot` (pitch rate) | MPU-6050 gyro rate — complementary-filtered with offset correction |
| `x_dot` (linear velocity) | Average wheel angular velocity × wheel radius (r = 0.034 m) |
| `x` (linear position) | Numerical integration of `x_dot` (updates every ≥ 5 ms) |

### Pin Mapping (Arduino Uno)

| Signal | Arduino Pin |
|---|---|
| Tachometer A (interrupt) | D2 |
| Tachometer B (interrupt) | D3 |
| Motor A PWM (ENA) | D5 |
| Motor A IN1 | D8 |
| Motor A IN2 | D9 |
| Motor B PWM (ENB) | D6 |
| Motor B IN3 | D10 |
| Motor B IN4 | D11 |
| MPU-6050 SDA | A4 |
| MPU-6050 SCL | A5 |

---

## Embedded Implementation (Arduino)

The control loop runs in `loop()` at a nominal **10 ms period** enforced by a `millis()` timer gate. State variables are sampled every iteration, but the control action `u` is computed and applied only when 10 ms have elapsed.

**Control law in code:**
```cpp
u = -( x*k1 + v_lineal*k2 + Pitch_angle*k3 + Pitch_rate*k4 );
// k = [-4.7152, -3.3278, -17.0580, -1.8229]
```

**Motor direction logic:**
```
u >  1  →  both motors BACKWARD  (robot tilted forward → roll back to correct)
u < -1  →  both motors FORWARD   (robot tilted backward → roll forward to correct)
|u| ≤ 1 →  both motors STOP      (near equilibrium)
```

PWM magnitude is clamped to **[0, 230]** (out of 255) to protect the motors.

**Startup sequence:**
1. Serial port opened at 115200 baud.
2. Tachometers and I²C bus initialised.
3. Gyro calibration (2000-sample average).
4. 3-second wait → position the robot upright before control begins.

---

## Repository Structure

```
SelfBalanceRobot/
│
├── README.md                              ← This file
├── ResumenEjecutivo.tex                   ← Full technical report (LaTeX source, Spanish)
├── TPF_SCD_RobotBalancin.pdf              ← Compiled technical report (Spanish)
├── TPF_SCD_RobotBalancin_EN.pdf          ← Compiled technical report (English)
│
├── 1- Matlab/
│   └── Discomp_SelfBalanceRobot.m           ← Plant modelling, ZOH discretisation,
│                                            LQR design & closed-loop simulation
│
├── 2- Arduino/
│   ├── main_SelfBalanceRobot/               ← Main sketch + all custom libraries
│   │   ├── main_SelfBalanceRobot.ino        ← Entry point (setup + loop)
│   │   ├── Gyro.h / Gyro.cpp            ← MPU-6050 driver v1 (Kalman filter)
│   │   ├── Gyro_MPU.h / Gyro_MPU.cpp   ← MPU-6050 driver v2 (production, with rates)
│   │   ├── Taco_twoPin.h / .cpp         ← Interrupt-driven optical encoder class
│   │   ├── L298N_driver_two_motor.h/.cpp ← L298N dual-motor H-bridge driver class
│   │   ├── VelocityIntegrator.h         ← Numerical velocity→position integrator
│   │   ├── ControlD.h                   ← Discrete state-feedback controller class
│   │   └── PitchSensor.h               ← Pitch sensor abstraction helper
│   │
│   └── Drivers_and_Sensors_Class/       ← Individual driver test sketches
│       ├── Driver_L298N_Class_test/
│       ├── Full_integration_of_test_classes/
│       ├── Gyro_Class_test/
│       ├── Gyro_MPU_classTest/
│       └── Taco_Class_test/
│
├── 3- Latex ZIP Projects/               ← LaTeX project source archives
│
├── 4- Multimedia/                       ← Photos and demo videos of prototype "Paco"
│   ├── Paco1.jpg
│   ├── Paco2.jpg
│   ├── SelfBalanceRobot_videoDemo.mp4   ← Main demonstration video
│   └── (additional demo videos)
│
└── 5- References/                       ← Bibliography PDFs
    ├── ECORFAN_Revista_de_Ingeniería_Eléctrica_VI_N3_1.pdf
    └── RobotBalancn2.pdf
```

---

## Software Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                   main_SelfBalanceRobot.ino                     │
│                                                              │
│  ┌─────────────┐   ┌──────────────┐   ┌───────────────────┐ │
│  │  Gyro_MPU   │   │  Taco × 2    │   │ VelocityIntegrator│ │
│  │ (MPU-6050)  │   │ (Encoders)   │   │  x = integral(v)  │ │
│  │ Kalman 1D   │   │  ISR-driven  │   └────────┬──────────┘ │
│  └──────┬──────┘   └──────┬───────┘            │ x          │
│     θ, θ_dot          ω_wheels                 │            │
│         └────────────────┬─────────────────────┘            │
│                          │  State vector X = [x, ẋ, θ, θ̇]  │
│               ┌──────────▼──────────────┐                   │
│               │    LQR Control Law      │                   │
│               │    u = -K * X           │                   │
│               │  K = [-4.71, -3.33,     │                   │
│               │        -17.06, -1.82]   │                   │
│               └──────────┬──────────────┘                   │
│                          │  u  (PWM command, clamped 0-230)  │
│               ┌──────────▼──────────────┐                   │
│               │  L298N_driver_two_motor │                   │
│               │  Motor A  /  Motor B    │                   │
│               └─────────────────────────┘                   │
└──────────────────────────────────────────────────────────────┘
```

### Custom C++ Class Summary

| Class | File(s) | Responsibility |
|---|---|---|
| `Gyro_MPU` | `Gyro_MPU.h/.cpp` | Reads MPU-6050 via I²C; applies 1-D Kalman filter (gyro + accel fusion); provides roll/pitch/yaw angles and rates |
| `Gyro` | `Gyro.h/.cpp` | Earlier IMU driver version used during sensor development and testing |
| `Taco` | `Taco_twoPin.h/.cpp` | Interrupt-driven optical encoder; calculates RPM and angular velocity with hardware debouncing (5 µs) |
| `L298N_driver_two_motor` | `L298N_driver_two_motor.h/.cpp` | Abstracts L298N H-bridge; exposes per-motor forward/backward/stop with PWM speed |
| `VelocityIntegrator` | `VelocityIntegrator.h` | Numerical integrator: accumulates `x += v * dt` with minimum 5 ms update guard |
| `ControlD` | `ControlD.h` | Header-only discrete controller; stores G, H, K matrices; includes optional Luenberger observer structure |

---

## Results

### MATLAB Simulation

Closed-loop response from an initial tilt of **θ₀ = 10° (0.1745 rad)** with all other states at zero.
The LQR controller drives all four states to equilibrium within approximately **2–3 seconds**.

| State | Description | Converges to |
|---|---|---|
| x₁ | Linear position [m] | 0 m |
| x₂ | Linear velocity [m/s] | 0 m/s |
| x₃ | Pitch angle [rad] | 0 rad |
| x₄ | Pitch angular rate [rad/s] | 0 rad/s |

### Physical Prototype

The robot **"Paco"** successfully self-balances in place and recovers from small manual disturbances. Pitch angle data logged via Serial confirms the system oscillates around **θ ≈ 0°** within the sensor noise floor. The absence of integral action results in a small but bounded steady-state position drift — a known characteristic of pure state-feedback without an integrating action.

Demo videos are available in the [`4- Multimedia/`](4-%20Multimedia/) folder.

---

## How to Run

### MATLAB Simulation

1. Open MATLAB (R2021a or newer recommended).
2. Navigate to the `1- Matlab/` folder.
3. Run `Discomp_SelfBalanceRobot.m`.
4. The script will:
   - Compute plant parameters from component masses and geometry.
   - Build the continuous-time state-space model (A, B, C, D).
   - Discretise with ZOH at Ts = 10 ms.
   - Verify controllability and observability.
   - Compute the discrete LQR gain K.
   - Simulate the closed-loop response and plot all four states.

### Arduino Firmware

**Requirements:** Arduino IDE 1.8+ or Arduino IDE 2.x, built-in Wire library.

1. Open `2- Arduino/main_SelfBalanceRobot/main_SelfBalanceRobot.ino` in the Arduino IDE.
2. Confirm all `.h` and `.cpp` files are present in the same folder.
3. Select **Board: Arduino Uno** and the correct COM port.
4. Upload the sketch.
5. The robot will:
   - Calibrate the MPU-6050 gyroscope (2000-sample average).
   - Wait 3 seconds — **stand the robot upright during this window**.
   - Begin the 10 ms control loop.

---

## References

1. Katsuhiko Ogata. *Discrete-Time Control Systems*, 2nd ed., Prentice Hall, Upper Saddle River, NJ, 1995.
2. Ramírez-González, L., García-Martínez, J., Siordia-Vásquez, X. & Garcia-Ramos, R. (2017). *Euler-Lagrange en el modelado y control de un péndulo invertido sobre un carro*. Revista de Ingeniería Eléctrica, ECORFAN, 1(3), 1–8.

---

## Demo Video

<p align="center">
  <a href="https://youtu.be/RLYL08lZvbI" target="_blank">
    <img src="https://img.youtube.com/vi/RLYL08lZvbI/hqdefault.jpg" width="70%" alt="Watch the demo on YouTube"/>
  </a>
</p>
<p align="center">
  <strong>▶ <a href="https://youtu.be/RLYL08lZvbI">Watch the full demo on YouTube</a></strong>
</p>

---

## License

This project was developed as an academic final project at Universidad Nacional de Asunción — CITEC-Luque, Paraguay. All code and documentation are provided for **educational purposes only**.
