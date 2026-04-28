# Networked Robotic Arm Control — Delay-Aware Adaptive Control in C++

> **Academic project** · Polytech Montpellier · Automatique & Réseaux S8 · Apr. 2026  
> Authors: Lucas Musumeci, Juliette Couderc  
> Full report: [`Rapport_Autom_Reseaux.pdf`](./Rapport_Autom_Reseaux.pdf)

---

## Overview

This project studies the impact of **network-induced delay** on the stability of a closed-loop robotic arm controller, and derives an **adaptive gain law** that keeps the system stable regardless of how large the round-trip delay is.

The physical plant is the **Robotis-H 6-DoF manipulator** simulated in CoppeliaSim. The software architecture deliberately separates the controller, the delay emulator, and the simulator interface into three independent processes communicating over **UDP sockets** — a realistic model of networked control over an unreliable link.

The project has three independent parts that build on each other:

1. **Networking & delay emulation** — three-process UDP architecture with a configurable delay block and Wireshark-verified message tracing.
2. **Motor identification & stability analysis** — per-joint first-order + pure delay model identified from velocity step responses, analytical stability boundary derived, adaptive gain law implemented and validated experimentally in three gain regimes.
3. **C++ robotics library** — a clean port of the Julia kinematics work (DH model, Jacobian, trapezoidal trajectory, resolved-rate control, null-space joint-limit avoidance) into an object-oriented C++ library using Eigen, designed to be simulator-agnostic.

---

## Repository Structure

```
.
├── clients_serveurs/
│   ├── client_robot/
│   │   ├── client_robot.cpp   # Controller: adaptive gain law, UDP client, CSV logger
│   │   ├── logger.hpp         # Lightweight CSV recorder (Logger class)
│   │   └── Makefile
│   ├── retard_robot/
│   │   └── retard_robot.c     # Delay emulator: buffers packets, re-sends after DELAY_US
│   └── serveur_robot/
│       ├── serveur_robot.cpp  # Simulator interface: CoppeliaSim API ↔ UDP bridge
│       └── Makefile
│
├── identification/
│   ├── step_ident.cpp         # Per-joint velocity step experiment, logs to CSV
│   ├── logger.hpp             # StepLogger class (cmd + measured velocity)
│   └── Makefile
│
├── MATLAB/
│   ├── identification_1erOrdre.m      # 1st-order + delay model fitting (63% method)
│   ├── courbe_stabilite_1erOrdre.m   # Stability boundary plot Trc_max vs Kc
│   └── plot_adaptative_control.m     # Dual-axis position & velocity response plots
│
├── robot/
│   ├── robManip.hpp           # Robot class: DH model, kinematics, trajectory, control
│   └── robManip.cpp           # Implementation (MGD, Jacobian, Trapeze, cmdCinematique)
│
├── Scene-robotis-crayon-prjrobS7-4.4-Velocity.ttt  # CoppeliaSim scene
└── Rapport_Autom_Reseaux.pdf
```

---

## Architecture

Three processes run concurrently and communicate via non-blocking UDP sockets:

```
┌──────────────────┐       CMD :2002        ┌──────────────────┐       CMD :2001       ┌─────────────────────┐
│  client_robot    │ ─────────────────────► │  retard_robot    │ ────────────────────► │  serveur_robot      │
│  127.0.0.3:2003  │                        │  127.0.0.2:2002  │                       │  127.0.0.1:2001     │
│                  │ ◄──────────────────────────────────────────────────────────────── │  + CoppeliaSim API  │
└──────────────────┘       State :2003                                                 └─────────────────────┘
```

Each command packet (`msg_t`, 168 bytes with compiler padding) carries:

```c
typedef struct {
    int    cmdType;           // POSITION or VELOCITY  (+4 bytes padding for alignment)
    double cmd[6];            // Joint command
    double q_simu[6];         // Measured joint positions (filled by server)
    double qdot_simu[6];      // Measured joint velocities
    struct timeval time;      // Client-side timestamp — used to measure Trc
} msg_t;
```

The **delay emulator** (`retard_robot.c`) uses a 1000-slot timestamp buffer to hold packets until their scheduled send time. The delay is set to 200 ms by default (`DELAY_US 200000`). It can be changed to a random distribution by uncommenting the relevant line.

The **server** (`serveur_robot.cpp`) runs in synchronous CoppeliaSim mode, triggers one simulation step per received command, reads back joint positions via streaming, and returns the full `msg_t` (with updated `q_simu`) to the client — crucially preserving the original `time` field so the client can measure the true round-trip delay `Trc`.

---

## Part I — Motor Identification

### Experiment

`step_ident.cpp` connects **directly to CoppeliaSim** (bypassing the delay block) and runs a per-joint velocity step experiment at dt = 1 ms. For each joint:

1. Short zero-velocity rest phase (5 ms).
2. Velocity step of amplitude `V_STEP` (0.05–0.1 rad/s) held for 40 ms.
3. Command and measured velocity logged to `joint{N}_step.csv`.

Using simulation time ensures sub-millisecond timing accuracy regardless of CPU load.

### MATLAB fitting — `identification_1erOrdre.m`

The script detects the step onset and the output reaction onset automatically, then fits a first-order model using the 63% rise-time method:

```
G(s) = K · e^(-Trc·s) / (τs + 1)
```

| Joint | K (G) | τ (s) |
|-------|-------|-------|
| 1     | 1.00  | 0.002 |
| 2     | 1.00  | 0.002 |
| 3     | 1.00  | 0.003 |
| 4     | 1.00  | 0.002 |
| 5     | 1.00  | 0.001 |
| 6     | 1.00  | 0.001 |

---

## Part II — Stability Analysis & Adaptive Control

### Stability boundary

The controller sends **velocity commands** to position the arm. The open-loop transfer function is therefore:

```
H_BO(s) = Kc · G / (s(1 + τs)) · e^(-Trc·s)
```

Setting the total phase lag to −π and applying the small-angle approximation (valid for τ ≪ Trc) yields the critical frequency and maximum stable gain:

```
ωu ≈ π / (2(Trc + τ))

K_max(Trc) = (ωu / G) · √(1 + (τ·ωu)²)
```

The system is **unconditionally stable** for Kc < 1/G. Above that threshold, stability depends on Trc. `courbe_stabilite_1erOrdre.m` plots the stability boundary for any (τ, G) pair.

### Adaptive gain law

The client measures `Trc` dynamically on every control cycle by comparing the `time` timestamp in the server's response against the current wall clock. It then recomputes `Kc` before sending each command:

```c
double omega_u = M_PI / (2.0 * (Trc + tau[i]));
double K_max   = (omega_u / G[i]) * sqrt(1.0 + pow(tau[i] * omega_u, 2));
double Kc      = alpha * K_max;   // alpha is the safety margin
```

### Experimental validation (200 ms emulated delay)

| α    | KcG  | Observed behaviour |
|------|------|--------------------|
| 0.4  | < 1  | Smooth convergence, slight overshoot, no oscillations |
| 1.1  | > 1  | Bounded oscillations — nonlinear velocity/acceleration limits act as energy dissipators, system eventually converges |
| 1.5  | ≫ 1  | Divergent oscillations — command and position near anti-phase; joints hit mechanical limits |

The α = 1.1 case is an interesting edge case: the linear model predicts instability, but the velocity clamping in `robManip.cpp` injects enough nonlinear damping to recover. At α = 1.5 the energy injected per cycle exceeds the dissipation capacity and the system truly diverges.

---

## Part III — C++ Robotics Library (`robManip`)

The `Robot` class in `robot/robManip.hpp` is a clean C++ port of the Julia kinematics developed in the [Robotis-H project](https://github.com/lucasmusumeci/6DoF_Robotis-H_Manipulator) and [KUKA LWR project](https://github.com/lucasmusumeci/7Dof_KUKA_LWR), using **Eigen** for all matrix operations.

### Key design choice — I/O abstraction

`sendCmd()` and `getAllJointsPosition()` are declared in the header but **not** implemented inside the library. The user provides them in their own translation unit (e.g. `client_robot.cpp` for UDP, `step_ident.cpp` for direct CoppeliaSim API calls). This makes `robManip` simulator-agnostic and portable to real hardware without touching the kinematics code.

### Robot class API

```cpp
// Forward kinematics
std::pair<Matrix4d, std::vector<Matrix4d>> MGD() const;

// Geometric Jacobian
MatrixXd Jacobienne(const Vector3d& P) const;

// Trapezoidal trajectory planning
Trapeze calculTrapeze(const VectorXd& qi, const VectorXd& qf, double duree) const;
MatrixXd calculQ(const VectorXd& qi, const Trapeze& trap, const VectorXd& t) const;
MatrixXd calculQdot(const VectorXd& qi, const Trapeze& trap, const VectorXd& t) const;

// Simulation methods (call sendCmd / getAllJointsPosition internally)
void simuTrapeze(int clientID, int *handles, const VectorXd& qf,
                 double duree, double dt, CmdType_t cmdType);
void cmdCinematique(int clientID, int *handles,
                    const Vector3d& Pd, const Matrix3d& Ad,
                    const Vector3d& dPd, const Vector3d& omega_d,
                    double Kp, double K0, double dt,
                    double alpha, double lambda_L, CmdType_t cmdType);

// Null-space joint-limit avoidance
VectorXd eloignement_butees_articulaires(const MatrixXd& J,
                                          const VectorXd& Xdot,
                                          double alpha) const;
```

`CreateRobotisH(initialTheta)` instantiates the Robotis-H 6-DoF model with the correct DH parameters, joint limits, and pencil tool transform out of the box.

---

## Getting Started

### Prerequisites

- Linux (tested on Debian)
- `g++` with C++17
- [Eigen 3](https://eigen.tuxfamily.org/) (`apt install libeigen3-dev`)
- CoppeliaSim 4.4 EDU with legacy Remote API enabled on port 5555
- MATLAB (for identification and plotting scripts)

### Build

Each module has its own Makefile:

```bash
# Delay emulator
cd clients_serveurs/retard_robot && gcc retard_robot.c -o retard_robot

# Simulator server
cd clients_serveurs/serveur_robot && make

# Controller client
cd clients_serveurs/client_robot && make

# Identification tool
cd identification && make
```

### Running

Launch the three processes in separate terminals in this order:

```bash
# Terminal 1 — start CoppeliaSim and open the scene, then:
./serveur_robot

# Terminal 2
./retard_robot

# Terminal 3
./client_robot
```

MATLAB plots are generated from the CSV logs produced by `client_robot` and `step_ident`:

```matlab
% After running step_ident to generate joint{N}_step.csv:
identification_1erOrdre   % fit one joint at a time (edit filename inside)

% After running client_robot to generate adaptive_control_log.csv:
plot_adaptative_control

% Stability boundary (no data needed):
courbe_stabilite_1erOrdre
```

### Key parameters to tune

In `client_robot.cpp`:

```cpp
double alpha = 0.4;          // Safety margin (0.2–0.6 for stable; >1 for instability demos)
int timeout_ms = 18000;      // Max time to reach target (ms)
double max_err_treshold = 0.001; // Convergence threshold (rad)
```

In `retard_robot.c`:

```c
#define DELAY_US 200000      // Network delay in microseconds (200 ms default)
```

---

## Project Report

Full derivations, Wireshark captures, stability plots, step response fits, and experimental results are in [`Rapport_Autom_Reseaux.pdf`](./Rapport_Autom_Reseaux.pdf).

---

## Related Projects

This project reuses and extends the kinematics and control laws developed in:

- [6DoF Robotis-H Manipulator](https://github.com/lucasmusumeci/6DoF_Robotis-H_Manipulator) — forward kinematics, trapezoidal trajectory, resolved-rate control (Julia)
- [7DoF KUKA LWR](https://github.com/lucasmusumeci/7Dof_KUKA_LWR) — redundancy, null-space joint-limit avoidance (Julia)

---

## Authors

**Lucas Musumeci** & **Juliette Couderc** — Polytech Montpellier, MEA department
