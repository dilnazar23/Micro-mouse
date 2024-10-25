# Lab07: Autonomous Driving

## Learning Outcome

- Know how to drive a robot autonomously on a higher-level.
- Understand how inverse kinematics is relevant to robot driving.
- Can apply a PID controller for positional closed-loop control.

---

## Prerequisites

### Related Lectures

- Kinematics.

### Prelab Video

Mandatory:
- [Inverse kinematics](https://unsw.sharepoint.com/:v:/s/CLS-MTRN3100_T2_5236_Combine/EV8oh37hurxIraOyBUOCUZEB9loQdHA9KYUW5EY7Qocl9A?e=OGDPuE)


### Kit

This lab will require:

- $1 \times$ robot assembled with:
    - $1 \times$ Arduino MEGA
    - $2 \times$ motors with encoders
    - $1 \times$ motor driver
    - $1 \times$ battery pack
    - $1 \times$ IMU

---

## Prelab

1. (2 marks) Create an inverse kinematic model for a robot undertaking purely linear motion where the input of the model is the next pose: `[x1, y1]`; and the output of the model are the wheel positions: `[thetaL, thetaR]`.

    Let:
    - `[x0, y0]` be the current pose.
    - `[x1, y1]` be the next pose.
    - `R` be the wheel radius.


1. (2 marks) Create an inverse kinematic model for a robot undertaking purely rotational motion where the input of the model is the next heading: `[h1]`; and the output of the model are the wheel positions: `[thetaL, thetaR]`.

    Let:
    - `h0` be the current heading.
    - `h1` be the next heading.
    - `L` be the axle length.
    - `R` be the wheel radius.


1. (1 marks) Identify a source of error for kinematic models of wheeled mobile robots.


---

## Lab

1. (required) Ensure the following libraries have been installed using the Arduino IDE library manager:
    - [ICM 20948 by SparkFun](https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary)

1. (required) Ensure the robot has been assembled using the circuit from lab04 for motor driver and IMU connections.

1. (required) Identify the axle length and wheel radius of the platform.

    These values will be required for inverse kinematics.


1. (5 marks) Open the `drive_by_pose` sketch. Implement this sketch so it drives a robot using inverse kinematics and open-loop control according to the following hardcoded sequence of poses.
    - `(0, 0, 0)`
    - `(1, 0, 0)`
    - `(1, 0, π/2)`
    - `(1, 0, π)`
    - `(0, 0, π)`
    - `(0, 0, π/2)`

    Validate the robot's position over time by inspection.

    The position is a vector in the global frame: `[x, y, h]`. The positive x-axis is where the heading is `0`. The positive y-axis is where the heading is `π/2`.

    The robot drives (in open-loop control) by using:
    - Inverse kinematics to determine the expected wheel positions.
    - Wheel encoder odometry to determine the actual wheel positions.
    
    The robot should stop when the expected/actual **wheel** positions are close or alternatively using forward kinematics to determine if the expected/actual **robot** positions are close.
    
    `delay()` for motor control is not allowed.

    Motions will either be linear or rotational - there will be no combination of linear and rotational motions in a single motion. You can assume that only one such motion will be required to change between consecutive poses.
    
    Note that the motions do not need to be accurate.


1. (5 marks) Copy the `drive_by_pose` sketch into the `closed_drive_by_pose` sketch. Modify the copied sketch by applying the PID controller onto the IMU i.e. the robot position directly to close the driving loop.

    Validate the robot's position over time by inspection.

    The inputs to the PID controller should be the expected and actual robot positions. The output of the PID controller is a PWM signal which can be used to drive the motors directly.

    You will need to copy your completed `IMU` and `PIDController` class into this sketch.

    **Note that if your IMU has not been working, then apply the PID controller onto the motor layer instead.**


1. (2 marks) Discuss the discrepancies in open-loop and closed-loop driving.


1. (required) Individually answer a demo question orally. The mark from this question will scale your lab mark.

