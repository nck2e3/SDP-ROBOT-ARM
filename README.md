# Face Tracking Robotic Arm

## Overview

This repository documents a Senior Design Project (SDP) centered around a robotic arm system. The project features **custom-developed firmware and hardware**, built to extend and optimize the functionality of an existing mechanical design. The full technical report can be found in the `Docs/` folder of this repository. This project was developed by **Junhee Lee** and **Noah King** as part of their Senior Design Project.

> ⚠️ Note: The mechanical CAD files—except for our custom-modeled housing—are sourced directly from the original forked repository, community_robot_arm, and remain unmodified. All electronic hardware and firmware in this repository are original to this project.

| Schematic                        |
|----------------------------------|
| ![](Docs/Images/schematic.svg)       |

| PCB Photo                        |
|----------------------------------|
| ![](Docs/Images/pcb_photo.jpg)       |

| Custom Housing                   | Entire Assembly                  |
|----------------------------------|----------------------------------|
| ![](Docs/Images/custom_case.png)     | ![](Docs/Images/assembly.jpg)         |

## Repository Structure

- `Firmware/` - Custom embedded firmware developed for motor control, sensing, and task coordination.
- `Hardware/` - Custom PCB schematics and layouts designed to interface with the robotic arm and support sensors and actuators.
- `Mechanical_Design/` - Original CAD files sourced from the forked project (see Acknowledgments).
- `Documentation/` - Design notes, test procedures, and system configuration details.

## Key Features

- Custom PCB design utilizing TMC-2209 motor drivers and an STM-32 microcontroller
- Face detection and tracking via external vision processing pipeline

## Acknowledgments

All firmware, hardware integration, and system-level enhancements were designed and implemented by the team.

Mechanical design elements, including all CAD files, originate from the [community_robot_arm](https://github.com/20sffactory/community_robot_arm) repository. These files were used unmodified and served as the foundation for the custom developments described in this repository.

