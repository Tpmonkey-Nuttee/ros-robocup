# Medical Guilding Bot

This scripts were written for [RoboCup@Home Education Workshop & Challenge 2022](https://www.robocupathomeedu.org/) at Thailand, KU

These scripts are not finished because we only have one day to write everything. but the plan was to made the bot be able to display image and wave hand.

## Running the scripts
> **_NOTE:_**  All commands need to be run on jupiter robot.

1. Make sure [ROS Package](https://github.com/robocupathomeedu/rc-home-edu-learn-ros) is installed.
2. Create a package in `~/catkin_ws/src` by using 
   ```bash
   catkin_create_pkg {pkg_name} std_msgs rospy roscpp
   ```
3. Go to `~/catkin_ws/src/pkg_name` and run
   ```bash
   mkdir scripts
   ```
4. Run git clone. (Can be run in another dir)
   ```bash
   git clone https://github.com/Tpmonkey-Nuttee/ros-robocup.git
   ```
5. Move all files out using
   ```bash
   mv /ros-robocup/bot/scripts /scripts
   ```
6. Make sure you have a map file ready and config `bot_nav2.py` to add/edit locations. 
7. Make sure files has execution permission.
   ```bash
    chmod +x /scripts/*.py
    ```
8. Run all the commands down below in difference terminals. (Recommend tabs)

    ```bash

    roscore
    roslaunch jupiterobot_bringup jupiterobot_bringup.launch

    roslaunch jupiterobot_navigation rplidar_amcl_demo.launch map_file:=/path/to/your/map.yaml

    # Don't forget to set robot current position.
    roslaunch turtlebot_rviz_launchers view_navigation.launch

    # To stop the bot from moving, terminate this terminal
    rosrun kus bot_nav2.py

    rosrun kus bot_speak.py
    rosrun kus bot_reg.py 

    roslaunch rchomeedu_vision multi_astra.launch
    roslaunch robot_vision_openvino yolo_ros.launch

    rosrun kus detect_chair.py 
    rosrun kus detect_human.py

    # To stop the bot cycle, terminate this terminal
    rosrun kus bot.py
    ```
9. Bot will start to navigate to elevator and the cylcle begins.
