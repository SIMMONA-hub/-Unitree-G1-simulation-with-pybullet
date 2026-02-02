# Unitree G1 Autonomous Simulation (PyBullet)

This repository presents an autonomous simulation system for the Unitree G1 humanoid robot.
The project was developed for the **Humanoid Robot Championship – Test Task: Search and Approach to an Object**.

The focus of this work is to demonstrate a complete autonomous pipeline — from perception to motion —
implemented under strict task constraints and without relying on simulator ground-truth shortcuts.

---

## Overview

The robot operates fully autonomously and must:

- search for a target object (cube)
- detect the object using visual perception
- orient itself toward the object
- approach it
- stop at a distance **≤ 0.5 meters**

All actions are performed without human input and without access to the true object position.

---

## Motivation and Approach

The task does not require highly dynamic humanoid locomotion.
Instead, it emphasizes autonomy, perception, and correct system design.

For this reason, the project prioritizes:
- clarity of control logic
- robustness to random initial conditions
- strict compliance with competition rules
- reproducibility and transparency

The solution avoids unnecessary complexity and focuses on a clean, interpretable pipeline.

---

## Simulator Choice

**PyBullet** is used as the primary simulator due to:

- fast iteration and debugging
- straightforward camera rendering
- simple scene construction from scratch
- sufficient physical realism for the task

Support for MuJoCo is included in the repository, but PyBullet is the main evaluation environment.

---

## Simulation Scene

The simulation environment is created manually and strictly follows the task specification:

- Room size: **4 × 4 meters**
- Square geometry
- Flat floor
- Static walls
- No obstacles
- Static lighting

No prebuilt scenes or world files are used.

---

## Target Object

- Type: cube
- Size: **0.3 × 0.3 × 0.3 meters**
- Placed on the floor
- Uniform color
- Position randomized between runs

The robot has **no access** to the object’s true position or simulator state.

---

## Robot Model

- Robot: **Unitree G1**
- Initial position: center of the room
- Initial orientation: randomized

To ensure stability and simplicity:
- the robot model is partially simplified
- locomotion is implemented via abstract base velocity control
- full humanoid walking dynamics are intentionally not modeled

This approach is explicitly allowed by the competition rules.

---

## Sensors and Perception

### RGB Camera
- The RGB camera is the **primary and mandatory sensor**
- Used for all object detection and navigation decisions

### Restrictions
- ❌ No ground-truth object coordinates
- ❌ No direct access to simulator object states
- ❌ No shortcuts via object IDs

All decisions are made solely based on camera images.

---

## Object Detection

A classical computer vision approach is used:

- RGB image acquisition from the simulated camera
- color-based segmentation
- contour detection
- bounding box extraction

From the bounding box:
- horizontal offset is used for orientation control
- apparent object size is used as an indirect distance cue

This approach is simple, interpretable, and robust enough for the task.

---

## Control Architecture

Robot behavior is implemented as a finite-state machine:

SEARCH → ALIGN → WALK → STOP 


### SEARCH
The robot rotates in place until the target object appears in the camera view.

### ALIGN
The robot rotates to center the object horizontally in the image.

### WALK
The robot moves forward while continuously correcting its orientation.

### STOP
The robot stops when the estimated distance to the object is **≤ 0.5 meters**.

All transitions occur automatically without user interaction.

---

## Entry Point

The entire system is launched from a single script:

```bash
python run_g1_pybullet.py
the scene is created

the robot is spawned

sensors are initialized

autonomous behavior starts immediately 

Design Principles

This project prioritizes:

autonomy over teleoperation

perception-based control over ground-truth access

simplicity over overengineering

reproducibility over hard-coded solutions

No machine learning is required; classical methods are sufficient for the task. 

Result

The robot reliably:

detects the object from different initial orientations

approaches it from various starting conditions

stops within the required distance

remains robust to changes in object placement

A short demonstration video (≤ 2 minutes) shows a full autonomous run. 

DEMO VIDEO
https://youtu.be/aEe11XYtmU4?si=jyXA0adDBxsaK0WF
