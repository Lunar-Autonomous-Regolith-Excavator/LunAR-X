# LX Autonomy Container

## Dockerfile
- Location: docker/lx_autonomy.Dockerfile
- Dockerfile installs an image with ROS 2 Humble
<br> <br>


## Running the code:
### Using the script and TMUX 
```
cd scripts
bash autonomy_terminal.sh
bash init_autonomy.sh
```

### Manual Launch
1. Start and enter the container with `docker-compose up --build`
2. Enter docker container using `docker exec -it <container_name> bash`
3. [ All the subsequent instructions are for inside the container ]
4. Change directory to `/home/lx_autonomy/lx_autonomy_ws/`
5. Run `colcon build`
6. Make sure `/joy` topic is being published (joystick is connected with `joy_linux` node running)
7. Run `. install/setup.bash`
8. Run `ros2 launch lx_bringup_autonomy bringup_autonomy.launch.py`
<br> <br>
