# LunAR-X
The LunAR-X codebase

## Instructions
### Initialization
1. Start and enter the container with `docker-compose up --build`
2. `bash scripts/lx.sh`
3. Before unlocking rover, press both joystick triggers to start joy data stream
4. Unlock rover using GUIDE button
5. Verify system functionality using LED status indicators and teleoperation
### Operation
1. TODO


## Joystick Layout
- GUIDE Button : Lock/Unlock rover
- START Button : Operation mode
- BACK Button : Task mode
- Left Stick : Mobility controls
- Right Stick : Linear actuator/lifting controls
- Left Trigger : Drum -ve
- Right Trigger : Drum +ve

## Development Notes
A few things to remember while developing code for this repository
1. No file (especially ros packages and source code files) should be created from inside the container. This is due to the restricted access of files created inside the container, they can only be edited from the editors inside the container and no changes done outside the container (eg. on host vscode) will be saved. Create all packages and possible files from the host vscode instead.
2. Every merge to devel should increase the version number mentioned in version.yaml. `feature/...` branches should increment by 0.1. `fix/...` branches should increment by 0.0.1.
3. Add version features to Drive > LX Documents > Software > Software Version after every version number increase.