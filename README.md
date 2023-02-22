# LunAR-X
The LunAR-X codebase

## Development Notes
A few things to remember while developing code for this repository
1. No file (especially ros packages and source code files) should be created from inside the container. This is due to the restricted access of files created inside the container, they can only be edited from the editors inside the container and no changes done outside the container (eg. on host vscode) will be saved. Create all packages and possible files from the host vscode instead.