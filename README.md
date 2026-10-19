# Robot-Core

Core backend code for Daydream Robotics' VEX U robots. The repository is being reorganized into separate V5 Brain and Jetson Nano projects, with protocol code shared between them.

## Repository structure

```text
Robot-Core/
├── v5-brain/       PROS project for the VEX V5 Brain
├── jetson-nano/    Jetson Nano MPC project and third-party dependencies
├── shared/         Shared protocol declarations
├── common-utils/   Plotting utilities and path test data
├── archive/        Historical and inactive code
├── docs/           MPC notes and the restructuring proposal
└── .github/        GitHub workflows
```

The Brain project keeps its current PROS-dependent motion and localization code together while the device boundary is established. The Nano project contains its MPC application and its own motion, localization, and serial interfaces. Shared protocol declarations live under `shared/include/daydream/shared/`.

Build instructions for each target are in its project directory. The Brain uses PROS Makefiles in `v5-brain/`; the Nano project uses `jetson-nano/Makefile` and its `common.mk` flags.
