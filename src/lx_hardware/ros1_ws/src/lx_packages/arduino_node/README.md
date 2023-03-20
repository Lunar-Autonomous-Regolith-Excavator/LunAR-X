1. Install Arduino IDE 2.0
2. Install Micro-ROS Arduino library from https://github.com/micro-ROS/micro_ros_arduino
    - Download the library zip file for ROS2 Humble
    - In Arduino IDE, go to Sketch -> Include Library -> Add .ZIP Library
    - If you select the Arduino Due board, it should automatically install the SAMD core
    - Patch the SAMD core (https://github.com/micro-ROS/micro_ros_arduino/tree/galactic#patch-samd)
    ```
    cd /home/<usr>/.arduino15/packages/arduino/hardware/sam/1.6.12

    curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/galactic/extras/patching_boards/platform_arduinocore_sam.txt > platform.txt
    ```
3. Uploading code
    - sudo chmod 777 /dev/ttyACM0
3. ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6
