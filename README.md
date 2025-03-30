<div align="center">

  <h1>AutoDrone</h1>
  
  <p>
    Work in progress: This repo contains all onboard, offboard, and testing scripts for an autonomous mapping drone created by Srinath Iyer. 
  </p>
  </div>



# Table of Contents:
- [Motivation:](#motivation)
- [Mechanical and Electrical Documentation:](#mechanical-and-electrical-documentation)
- [Stack:](#stack)
- [Onboard:](#onboard)
    - [Controls](#controls)
    - [Sensor Fusion and Pose Estimation](#sensor-fusion-and-pose-estimation)
    - [Path Planning](#path-planning)
    - [Vision](#vision)
    - [Mapping](#mapping)
- [Offboard:](#offboard)
    - [Simulations](#simulations)
    - [Path Planning](#path-planning-1)
    - [Command Interface](#command-interface)
- [Scripts:](#scripts)
- [Getting Started:](#getting-started)
- [Future Work:](#future-work)

Note: All things here are tentative, as they are being actively developed. However, we are updating this README as a way to document our methods and current progress.

# Motivation:

# Mechanical and Electrical Documentation:
[Coming soon]

# Onboard:

There are two different onboard controlers on this drone: an ESP-WROOM-32 and Raspberry Pi 4B (8GB). The ESP32 handles controls, primary sensor fusion/pose estimation, and also is on the recieving end of path-planning commands from the RPi. The RPi handles CV sensor fusion (see more in [Vision](#vision)), vision mapping, path-planning, and logging to the user. The ESP32 code is written with the ESP-IDF framework and takes advantage of FreeRTOS to concurrently run tasks at very high frequencies. 

## Controls:

The current controls consists of the following:
  - PID Controllers for Roll, Pitch, Yaw, and Thrust: 
    - State and position estimation is handled by [sensor fusion](#sensor-fusion-and-pose-estimation), which is passed into the controllers in multiple ways.
  - Motor Mixing Algorithm to determine the power allocations for each motor
  - Software that directly sends PWM signals off the GPIO pins
  - Additional safety to ensure speed, voltage, and position tolerances.

## Sensor Fusion and Pose Estimation:

Arguably the most crucial aspect of the drone, sensor fusion allows the drone to take direct inputs from the IMU and camera, and combine and filter readings in different ways to produce a more confident pose estimation. This pose estimation is crucial because it enables the drone to fly autonomously in a precise and accurate manner. There are two ways that we achieve sensor fusion, and more detail can be found in the RPi's and ESP32's `sensor_fusion` directories:

  - Onboard ESP-32: Since speed of tasks is crucial to running controls on the ESP32 (the main controls tasks runs at 500 Hz), the sensor fusion method here takes the readings from the IMU and 
  - Onboard RPi: The RPi runs at a slower rate than the ESP32, making it a bad real-time motor controller. However, since it has lots of memory and processing power, it runs the vision stack. We are currently planning on using CV to correct the IMU sensor fusion (at 100 Hz) estimates, as it doesn't have the same drift issues. We plan to overwrite the ESP32's pose estimation for every iteration of the CV pose estimation, but are still developing the specifics of this plan. 

While this method would result in a small delay between the actual pose and the pose used for controls, we plan to tune PID to be able to overcome this. Testing will be required to see if this method is worth doing.

## Path Planning:

[Coming Soon]

## Vision:

Vision is a crucial part of the drone; it allows the drone to perceive, classify, and learn more about objects in its surroundings. This code is executed on the Pi, and currently uses a Pi Camera Module 3.

## Mapping

[Coming Soon]

# Offboard

Offboard code consists of code that is 

## Simulations

[Coming Soon]

## Path Planning

[Coming Soon]

## Command Interface

[Coming Soon]

# Scripts

Scripts are small snippets of test code used for benchmarking purposes. They're not actually used in the final drone system, but are there mostly because they may be of help to anyone who wants to try to replicate this process. Additionally, they are helpful snippets to demonstrate basic logic/basic structure for more complex onboard code.

# Getting Started:

# Future Work:


