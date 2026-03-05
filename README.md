# PID-Controller-for-Drone-Navigation-in-Gazebo-with-ROS2

A practical implementation of a PID (Proportional-Integral-Derivative) controller applied to autonomous drone navigation in a 3D simulated environment. This repository documents the full development pipeline, from ROS2 node design and position sampling to manual gain tuning, trajectory tracking, and multi-axis telemetry visualization using the Parrot AR Drone package inside Gazebo.
<br>


## 📋 Table of Contents
1. [Overview & Motivation](#overview--motivation)
2. [Objectives](#objectives)
3. [Tools & Environment](#tools--environment)
4. [System Architecture — Inputs, Outputs & General Operation](#system-architecture--inputs-outputs--general-operation)
5. [Controller Design](#controller-design)
6. [PID Gain Tuning — Manual Synchronization](#pid-gain-tuning--manual-synchronization)
7. [Final PID Configuration](#final-pid-configuration)
8. [Pipeline Structure](#pipeline-structure)
9. [Class & Function Descriptions](#class--function-descriptions)
10. [Desired Target Positions (4 Cubes)](#desired-target-positions-4-cubes)
11. [Experimental Results](#experimental-results)
12. [Version Control Log](#version-control-log)
13. [File Descriptions](#file-descriptions)
14. [Reproducibility](#reproducibility)
15. [References](#references)
<br>


## Overview & Motivation
Traditional manual drone control relies on real-time human input and is highly susceptible to position drift, latency, and operator error. Autonomous navigation through closed-loop feedback control addresses these limitations by continuously computing corrections based on position error.

This project implements a **3D PID controller** integrated with ROS2 and the Gazebo simulation engine. The drone receives real-time position feedback from the `/simple_drone/gt_pose` topic and adjusts its velocity commands simultaneously across the X, Y, and Z axes to converge toward a desired target position defined in a CSV file.

The full system was designed, tested, and validated against 4 target cube positions inside a Gazebo world, generating automated telemetry reports and multi-axis visualizations per experiment.
<br>


## Objectives
### General:
Design, implement, and validate a PID-based position controller for autonomous drone navigation within a Gazebo simulation environment using ROS2 as the communication middleware.

### Specific:
1. Implement ROS2 node architecture to subscribe to real-time drone pose data and publish velocity commands via the `cmd_vel` topic.
2. Design a CSV-based I/O system for position capture, desired target injection, and telemetry logging.
3. Manually tune PID gains (Kp, Ki, Kd) to achieve stable convergence under a defined distance threshold and velocity constraint.
4. Generate automated multi-axis plots for position, velocity, error, integral, and derivative signals per iteration.
5. Validate the controller against 4 distinct target positions (cubes) within the simulation world.
<br>


## Tools & Environment
| Tool / Library | Role in Project |
|----------------|----------------|
| **Ubuntu 22.04 LTS** ("Jammy Jellyfish") | Base operating system for all ROS2 and simulation operations |
| **ROS2 (Humble Hawksbill)** | Middleware layer for inter-process communication between simulation and control script |
| **Gazebo** | 3D open-source robot simulator; handles physics, collision, and drone dynamics |
| **Parrot AR Drone Package (ROS2)** | ROS2 package emulating the physical behavior and 3D model of the drone in Gazebo |
| **ImageViewer Package (ROS2)** | Provides first-person visual perspective from the drone model inside Gazebo |
| **Keyboard Package (ROS2)** | Originally used for manual drone validation; adapted as the base for `Control_Drone` class |
| **VSCodium** | Open-source IDE (VS Code without telemetry); used for iterative code execution via Jupyter-style blocks |
| **Pandas** | DataFrame operations for CSV-based position capture, telemetry logging, and structured output |
| **Matplotlib** | Automated generation of 5 multi-axis telemetry graphs per experiment |
| **Glob** | Regex-based file detection for automatic CSV discovery across dynamic directory structures |
| **NumPy** | Numerical operations used in PID computation and array handling |
| **Math** | Euclidean distance calculation between current and desired 3D position |
<br>


## System Architecture — Inputs, Outputs & General Operation
The controller operates over a continuous feedback loop driven by two CSV file sources and a ROS2 subscription:
<br>
```
┌──────────────────────────────────────────────────────────────────┐
│                     PID CONTROL LOOP                             │
│                                                                  │
│  ┌──────────────┐    ┌────────────────┐    ┌──────────────────┐ │
│  │ DESIRED_POS  │    │ CURRENT_POS    │    │ Control_Drone    │ │
│  │ (CSV Input)  │───▶│ (ROS2 + CSV)   │───▶│ D_Move_All(x,y,z)│ │
│  └──────────────┘    └────────────────┘    └──────────────────┘ │
│                              │                       │           │
│                         Error (e)           Velocity command     │
│                         Integral (∑e·dt)    published to         │
│                         Derivative (Δe/dt)  /simple_drone/cmd_vel│
│                              │                                   │
│                     ┌────────▼────────┐                          │
│                     │  FINAL_POSITION │                          │
│                     │  (CSV Output)   │                          │
│                     │  GRAPHICS/      │                          │
│                     │  (PNG outputs)  │                          │
│                     └─────────────────┘                          │
└──────────────────────────────────────────────────────────────────┘
```
The `CURRENT_POSITION` folder is continuously rewritten with average sampled positions from the ROS2 subscriber. The `DESIRED_POSITION` folder contains a static CSV with the target coordinates. The `FINAL_POSITION` folder stores the full telemetry log post-convergence, and `GRAPHICS/` stores 5 automatically generated PNG plots.
<br>


## Controller Design
### Controller Type: **PID (Proportional-Integral-Derivative)**
The PID control law applied simultaneously across the three spatial axes is:
```
adjustment = Kp·e(t) + Ki·∫e(t)dt + Kd·(Δe/dt)
```
| Component | Signal | Description |
|-----------|--------|-------------|
| **Proportional (P)** | `Kp · error` | Drives the drone toward the target linearly; directly responsive to current position error |
| **Integral (I)** | `Ki · Σ(error · dt)` | Compensates for persistent steady-state error by accumulating historical error |
| **Derivative (D)** | `Kd · (error - prev_error) / dt` | Damps oscillations by reacting to the rate of change of error |
| **Anti-windup** | Velocity magnitude condition | Integral is only accumulated when `v_x² + v_y² + v_z² < v_max²` to prevent divergence |
<br>

The convergence condition is defined as reaching a Euclidean distance below **0.01 m** from the desired target.
<br>


## PID Gain Tuning — Manual Synchronization
Manual tuning was performed following a structured process of progressive adjustment:
<br>
| Step | Configuration | Observation |
|------|--------------|-------------|
| Initial | `Kp=variable`, `Ki=0`, `Kd=0` | Kp swept until oscillations appeared |
| Step 2 | `Kp=100` (half of oscillation value) | Convergence with heavy oscillation |
| Step 3 | Anti-windup added (`v_max²` condition) | Integral divergence controlled |
| Step 4 | Simulation capped at 50 iterations for Ki/Kd testing | Isolated gain effects per axis |
| Step 5 | `Kp` reduced to `1` | Linear acceleration normalized to sampling period `dt` |
| Step 6 | `Ki=0.1` tested | Smaller adjustments but drone movement impaired |
| Step 7 | `Kp=1, Ki=0.01, Kd=0.1` | Target not reached; abrupt Kd oscillations observed |
| Step 8 | `Kp=1, Ki=0.001, Kd=0.01` | **Target reached in 19 iterations** |
| Step 9 | `Kp=1, Ki=0.0001, Kd=0.01` | **Target reached in 26 iterations** (threshold = 0.01) |
| Step 10 | Same config, `v_max=0.3` | **Target reached in 20 iterations** |
| Step 11 | Same config, `dt=0.1` | Target reached in 30 iterations (slower sampling) |
<br>


## Final PID Configuration
```python
dt             = 0.01   # Sampling period (seconds)
k_p            = 1      # Proportional gain
k_i            = 0.0001 # Integral gain
k_d            = 0.01   # Derivative gain
velocidad_maxima = 0.3  # Maximum velocity constraint (m/s)
```
This configuration provided the best balance between convergence speed, stability, and oscillation damping across all 4 tested target positions.
<br>


## Pipeline Structure
1. ROS2 initialization and folder validation (`DESIRED_POSITION`, `FINAL_POSITION`, `GRAPHICS`, `CURRENT_POSITION`)
2. Test routine — takeoff, move in X, stop, land, re-takeoff, return — validates Gazebo connection
3. First position sample via `Get_Average_Actual_Position_ROS2()` (0.25s capture window, averaged)
4. Desired position loaded from `DESIRED_POSITION/*.csv`
5. Initial Euclidean distance computed between current and desired 3D position
6. PID control loop executes until distance < 0.01 m:
   - Compute error per axis
   - Compute derivative and integral (with anti-windup)
   - Publish velocity via `D_Move_All(v_x, v_y, v_z)`
   - Update sampled position and recalculate distance
7. Drone stops and lands on convergence
8. Full telemetry exported to `FINAL_POSITION/FINAL_POSITION.csv`
9. `MULTI_GRAPH()` generates 5 multi-axis PNG plots saved to `GRAPHICS/`
<br>


## Class & Function Descriptions
| Class / Function | Type | Description |
|-----------------|------|-------------|
| `Validate_Folder(path)` | Function | Checks if a directory exists; creates it if absent. Returns `True` if it already existed |
| `distancia(punto1, punto2)` | Function | Computes Euclidean distance between two 3D points `(x1,y1,z1)` and `(x2,y2,z2)` |
| `Position_Drone(duration)` | ROS2 Node | Subscribes to `/simple_drone/gt_pose`; collects position samples for `duration` seconds; writes to CSV; terminates via timer |
| `Get_Average_Actual_Position_ROS2()` | Function | Instantiates `Position_Drone` for 0.25 s, spins the node until alive=False, then reads the CSV and returns `(mean_x, mean_y, mean_z)` |
| `Control_Drone` | ROS2 Node | Publishes to `/simple_drone/takeoff`, `/simple_drone/land`, `/simple_drone/cmd_vel`; exposes `D_Takeoff()`, `D_Land()`, `D_Stop()`, `D_Move_One()`, `D_Move_All()` |
| `Get_Desired_Position()` | Function | Uses Glob to detect the CSV in `DESIRED_POSITION/`; returns `(x, y, z)` target coordinates |
| `Action_Bailesito_Prueba()` | Function | Startup validation routine: takeoff → move X-0.8 → stop → land → takeoff → move X+0.8 → stop → land |
| `MULTI_GRAPH(...)` | Function | Generates 5 dark-background multi-axis plots (Position, Velocity, Error, Integral, Derivative) vs iteration number; saves to `GRAPHICS/` at 1400 DPI |
<br>


## Desired Target Positions (4 Cubes)
The following cube positions were tested, with a Z offset of +3 m applied to account for drone takeoff clearance:
<br>
| Cube | X (m) | Y (m) | Z (m) | Status |
|------|--------|--------|--------|--------|
| **1** | -3.44 | -9.53 | 3.4 | ✅ Reached |
| **2** | -9.34 | 5.92 | 3.4 | ✅ Reached (Video recorded) |
| **3** | 4.34 | 9.36 | 3.4 | ✅ Reached |
| **4** | 9.51 | -4.51 | 3.4 | ✅ Reached |
<br>


## Experimental Results
All 4 cube targets were successfully reached using the final PID configuration. For each target, the following telemetry was captured and visualized:
<br>
| Figure | Description |
|--------|-------------|
| Figures 1, 5, 9, 13 | Desired coordinates for each cube target |
| Figures 2, 6, 10, 14 | Tabular telemetry excerpt from `FINAL_POSITION.csv` |
| Figures 3, 7, 11, 15 | **Position vs Iterations** (X=red, Y=green, Z=blue) |
| Figures 4, 8, 12, 16 | **Error vs Iterations** (X=red, Y=green, Z=blue) |
<br>


### Telemetry CSV Structure
The output file `FINAL_POSITION/FINAL_POSITION.csv` stores the following columns per iteration:
<br>
| Column | Description |
|--------|-------------|
| `I` | Iteration number |
| `K_P`, `K_I`, `K_D` | PID gain constants |
| `D` | Euclidean distance to desired position |
| `X`, `Y`, `Z` | Current drone position |
| `E_X`, `E_Y`, `E_Z` | Position error per axis |
| `I_X`, `I_Y`, `I_Z` | Accumulated integral per axis |
| `D_X`, `D_Y`, `D_Z` | Derivative term per axis |
| `V_X`, `V_Y`, `V_Z` | Velocity adjustment (PID output) per axis |
<br>


## Version Control Log
The following describes the most significant development milestones across file versions:
<br>
| Version | Description |
|---------|-------------|
| `Position_00.py` | Automated drone position capture in terminal via ROS2 |
| `Position_02.py` | Class-based manipulation of drone coordinate data |
| `Position_05.py` | ROS2 node with 2-second lifecycle writing position to CSV (10–20 samples) |
| `Position_09.py` | Debugged CSV writer; average position now computed directly from sampled CSV |
| `Controlador_PID_00.py` | `Validate_Folder()` implemented; terminal-based command testing |
| `Controlador_PID_10.py` | `SubDrone` class with duration-based position sampling |
| `Controlador_PID_15.py` | State-controlled capture; `Control_Drone` class introduced |
| `Controlador_PID_20.py` | Drone control tests with `pyautogui`, `keyboard`, `pyinput`, `xdotool` (discarded) |
| `Controlador_PID_27.py` | `Control_Drone` class functional for single-axis linear motion |
| `Controlador_PID_41.py` | `D_Move_One` and `D_Move_All` added; distance calculation corrected |
| `Controlador_PID_50.py` | Final CSV structure defined: `I, K_P, K_I, K_D, D, X, Y, Z, E_X…` |
| `Controlador_PID_54.py` | PID calculation corrected; previous version was functional but required excessive iterations |
| `Controlador_PID_60.py` | PID improvements validated; used for gain tuning with `dt=0.1` |
| `Controlador_PID_63.py` | **Final video version**: includes startup validation routine, full control loop, convergence detection, and auto-save |
| `Controlador_PID_64.py` | **Current version**: final refinements, full pipeline with automated graphing |
<br>


## File Descriptions
<br>
| File | Description |
|------|-------------|
| `Controlador_PID_64.py` | Main script containing the full PID controller pipeline, ROS2 nodes, telemetry logging, and graph generation. It generates all the other files. |
| `CONTROLADOR_PID_REPORTEv2.docx` | Full lab report documenting methodology, tuning process, experimental results, and figure references, redacted in spanish |
| `DESIRED_POSITION/*.csv` | Input CSV with the target X, Y, Z coordinates for the drone |
| `CURRENT_POSITION/CURRENT_POSITION.csv` | Auto-generated CSV with sampled position data during each 0.25 s capture window |
| `FINAL_POSITION/FINAL_POSITION.csv` | Complete per-iteration telemetry log (position, error, integral, derivative, velocity, gains, distance) |
| `GRAPHICS/P_vs_Ni.png` | Position vs Iterations plot (3 axes) |
| `GRAPHICS/V_vs_Ni.png` | Velocity vs Iterations plot (3 axes) |
| `GRAPHICS/E_vs_Ni.png` | Error vs Iterations plot (3 axes) |
| `GRAPHICS/I_vs_Ni.png` | Integral vs Iterations plot (3 axes) |
| `GRAPHICS/D_vs_Ni.png` | Derivative vs Iterations plot (3 axes) |
<br>


## Reproducibility
### Prerequisites
```bash
# ROS2 Humble Hawksbill (Ubuntu 22.04)
sudo apt install ros-humble-desktop

# Gazebo + Parrot AR Drone Package
# Follow: https://docs.ros.org/en/humble/index.html

# Python dependencies
pip install pandas matplotlib numpy
```

### Workspace Setup
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Launch Gazebo world with drone model
ros2 launch  .py
```

### Run the Controller
```bash
# Place desired coordinates in DESIRED_POSITION/desired.csv (columns: x, y, z)
# Then run:
python3 Controlador_PID_64.py
```

### Expected Directory Structure
```
project_root/
├── Controlador_PID_64.py
├── DESIRED_POSITION/
│   └── desired.csv
├── CURRENT_POSITION/
│   └── CURRENT_POSITION.csv       # Auto-generated
├── FINAL_POSITION/
│   └── FINAL_POSITION.csv         # Auto-generated after convergence
└── GRAPHICS/
    ├── P_vs_Ni.png
    ├── V_vs_Ni.png
    ├── E_vs_Ni.png
    ├── I_vs_Ni.png
    └── D_vs_Ni.png
```
<br>


## References
| # | Reference |
|---|-----------|
| 1 | "Pandas Library Documentation" [Online]. Available: https://pandas.pydata.org/docs/user_guide/index.html |
| 2 | "Matplotlib Library Documentation" [Online]. Available: https://matplotlib.org/stable/users/index |
| 3 | "Glob Library Documentation" [Online]. Available: https://docs.python.org/3/library/glob.html |
| 4 | "ROS2 Humble Hawksbill Documentation" [Online]. Available: https://docs.ros.org/en/humble/index.html |
| 5 | M.C. Leticia Oyuki Rojas Pérez, M.C. Aldrich Alfredo Cabrera Ponce, Dr. José Martínez Carranza (January 2024). "Instalación de ROS2, Ubuntu, Gazebo, Parrot AR, Keyboard, ImageViewer" |
| 6 | "Controlador PID digital" [Online]. Available: https://www.picuino.com/es/control-pid-digital.html |
| 7 | "Gazebo Documentation" [Online]. Available: https://gazebosim.org/docs |

