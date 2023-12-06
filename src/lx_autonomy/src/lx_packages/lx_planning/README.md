Steps to run task optimizer: \\
0. Build the package with symlink install
```
colcon build --packages-up-to lx_planning --symlink-install
```
1. Start the task optimizer node:
```
ros2 run lx_planning task_optimizer_node
```
2. Call the task optimizer node with the desired yaml file name to call the task optimizer with:
```
python3 src/lx_packages/lx_planning/src/call_task_optimizer.py small_circle (or any yaml file)
```
3. Run the animation script:
```
python3 src/lx_packages/lx_planning/animation/animate.py small_circle (or any yaml file)
```
Video is saved as `animation.mp4` in the bags folder
