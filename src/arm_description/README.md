# arm_description

This package contains the robot description for this repository.

## Single source of truth

- The authoritative robot description is the Xacro: `xacro/ur5.xacro`.
- URDF files are treated as generated artifacts (for example from `xacro`) and should not be edited or committed.

## Inertial model disclaimer

The inertial parameters in `xacro/ur5.xacro` are **simplified** and intended for simulation stability and controller evaluation.
They are not identification-grade dynamics and should not be used as-is for high-fidelity dynamic simulation.
