<div align="center">

  <h1>AutoDrone</h1>
  
  <p>
    This repo contains all onboard, offboard, and testing scripts for an autonomous mapping drone created by Srinath Iyer. 
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
    - [Offboard Comms](#offboard-comms)
- [Offboard:](#offboard)
    - [Simulations](#simulations)
    - [Path Planning](#path-planning-1)
    - [Command Interface](#command-interface)
- [Scripts:](#scripts)
- [Getting Started:](#getting-started)
- [Future Work:](#future-work)

# Motivation:

# Mechanical and Electrical Documentation:
[Coming soon]

# Onboard:

There are two different onboard controlers on this drone: an ESP-WROOM-32 and Raspberry Pi 4B (8GB). The ESP32 handles controls, sensor fusion/pose estimation, and also is on the recieving end of path-planning commands from the RPi. The ESP32 code is written with the ESP-IDF framework and takes advantage of FreeRTOS to concurrently run tasks at very high frequencies.

## Controls:

Controls consists of 

## Sensor Fusion and Pose Estimation:

Another crucial 

## Path Planning:

Path Planning is really the heart of the drone. Users specify

## Vision:

Vision is a crucial part of the drone; it allows the drone to perceive, classify, and learn more about objects in its surroundings. This code is executed on the Pi, and currently uses a Logitech C615. In the future, a Pi Camera Module 3 may be used instead because of lower weight.

## Mapping

Mapping is the process by which the drone 

## Offboard Comms


# Offboard

Offboard code consists of code that is 

## Simulations

## Path Planning

## Command Interface

# Scripts

Scripts are small snippets of test code used for benchmarking purposes. They're not actually used in the final drone system, but are there mostly because they may be of help to anyone who wants to try to replicate this process. Additionally, they are helpful snippets to demonstrate basic logic/basic structure for more complex onboard code.

# Getting Started:

# Future Work:


