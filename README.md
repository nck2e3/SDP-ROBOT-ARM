# SDP Robotic Arm

## Overview

This repository documents a Senior Design Project (SDP) centered around a robotic arm system. The project features **custom-developed firmware and hardware**, built to extend and optimize the functionality of an existing mechanical design.

> ⚠️ **Note:** The **mechanical CAD files** used in this project are **from the original forked repository** [community_robot_arm](https://github.com/20sffactory/community_robot_arm) and have not been modified. All hardware and firmware enhancements are original to this implementation.

## Repository Structure

- `Firmware/` - Custom embedded firmware developed for motor control, sensing, and task coordination.
- `Hardware/` - Custom PCB schematics and layouts designed to interface with the robotic arm and support sensors and actuators.
- `Mechanical_Design/` - Original CAD files sourced from the forked project (see Acknowledgments).
- `Documentation/` - Design notes, test procedures, and system configuration details.

## Key Features

- Modular firmware with multi-threaded processing pipelines
- Optimized motor driver logic with encoder feedback
- Power management system for actuator efficiency
- Face detection and tracking via external vision processing pipeline

## Acknowledgments

Mechanical design elements, including all CAD files, originate from the [community_robot_arm](https://github.com/20sffactory/community_robot_arm) repository. Our team is responsible for all custom firmware, hardware integration, and project-specific enhancements built on top of the original mechanical framework.

