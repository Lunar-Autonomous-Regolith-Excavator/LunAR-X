# LunAR-X
The LunAR-X codebase

## Instructions
### Joystick Layout
TODO

### lx_autonomy
1. Start and enter the container with `docker-compose up --build`
2. Enter docker container using `docker exec -it <container_name> bash`
3. [ All the subsequent instructions are for inside the container ]
4. Change directory to `/home/lx_autonomy/lx_autonomy_ws/`
5. Run `colcon build`
6. Make sure `/joy` topic is being published (joystick is connected with `joy_linux` node running)
7. Run `. install/setup.bash`
8. Run `ros2 launch lx_bringup_autonomy bringup_autonomy.launch.py`

## Development Notes
A few things to remember while developing code for this repository
1. No file (especially ros packages and source code files) should be created from inside the container. This is due to the restricted access of files created inside the container, they can only be edited from the editors inside the container and no changes done outside the container (eg. on host vscode) will be saved. Create all packages and possible files from the host vscode instead.