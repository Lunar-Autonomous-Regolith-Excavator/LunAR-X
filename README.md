# LunAR-X
The LunAR-X codebase

## Development Notes
A few things to remember while developing code for this repository
1. Only create the ros2 packages from inside the docker container, no other file (especially source code files) should be created from inside the container. This is due to the restricted access of files created inside the container, they can only be edited from the editors inside the container and no changes done outside the container (eg. host vscode) will be saved. Create all possible files from the host vscode instead.