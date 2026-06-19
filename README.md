# Robot-Core

Core backend code for Daydream Robotics' VEX U robots.

Robot-Core contains the shared robot systems that run on the V5 Brain, including autonomous logic, odometry, PID control, path following, subsystem control, helper utilities, and path files. This repository is intended to serve as the common backend foundation for both robots.

## Overview

Robot-Core is responsible for the low-level and mid-level robot functionality used across the team’s codebase:

- Robot lifecycle code
- Autonomous routines
- Odometry and localization
- PID control
- Pure pursuit / path following
- Path and spline utilities
- Subsystem interfaces
- Object handling
- SD card logging
- SLAM / positioning experiments

The repository currently includes `src`, `include`, `paths`, and `firmware` directories, along with PROS build files such as `project.pros`, `Makefile`, and `common.mk`.

## Repository Structure

```txt
Robot-Core/
├── include/        # Header files and public interfaces
├── src/            # Core robot source files
├── paths/          # Path files used by autonomous/path-following systems
├── firmware/       # PROS/V5 firmware libraries and linker files
├── Makefile        # Build configuration
├── common.mk       # Shared PROS make configuration
├── project.pros    # PROS project metadata
└── README.md
