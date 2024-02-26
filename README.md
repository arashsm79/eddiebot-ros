# Eddiebot ROS
> ROS2 packages for Parallax Eddie robot along with simulations using Gazebo (formerly ignition gazebo)


<p align="center"><img src="https://github.com/arashsm79/eddiebot-ros/assets/57039957/0ad1ecc0-379b-4573-b4df-2ff19c9f3b11"></p>

For an in depth tutorial on how to use this package check out the accompanying tutorial: [robotics lab](https://github.com/arashsm79/robotics-lab)

## Packages 
* [eddiebot_bringup](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_bringup): The driver for the Eddie robot board. It establishes a connection to the board using UART and acts as a bridge between ROS2 and Eddie commands that the firmware on the board understands. For more details about the commands please checkout the documentation directory.
* [eddiebot_description](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_description): This package defines structure of the robot using URDF modelsf for every component of the robot. These models are used for generating kinematic and tf information. It uses the `diff_drive` plugin for differential drive control of the robot. The launch files start `robot_state_publisher` that reads the URDF model and generates static TF transforms, and `joint_state_publisher` that sends out dynamic TF transforms for joint position updates.
* [eddiebot_gazebo](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_gazebo): Contains some sample worlds to test the robot in, along with a bunch of launch files that are able to setup Gazebo, spawn Eddie into the world, and necessary bridges for ROS2 and Gazebo communication.
* [eddiebot_nav](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_nav): Contains the launch files for the setup and launch of the navigation stack using nav2, slam_toolbox, depthimage_to_laserscan, and various other packages.
* [eddiebot_msgs](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_msgs): Definition of specific topics and services for Eddie robot.
* [eddiebot_odom](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_odom): Publishes odometry information useful for mapping and localization. It publishes the transform between the frames `/odom` and `/base_footprint`.
* [eddiebot_rviz](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_rviz): Rviz configurations and launch files for visualizing and monitoring various aspects of Eddie.
* [eddiebot_vel_controller](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_vel_controller): A converter between different velocity types.

## Screeenshots
![gaz](/assets/eddie-gazebo.png)
![rviz](/assets/rviz.png)
![maze](/assets/eddie-maze.png)
![nav](/assets/nav.png)

https://github.com/arashsm79/eddiebot-ros/assets/57039957/af5bc3b8-e1a9-4ce3-84b5-ed4566440294


## Features
- How to convert a legacy ROS1 codebase to ROS2
- Example of hooks for setting GZ_SIM_RESOURCE_PATH
- Robot description launch with robot_state_publisher, joint_state_publisher, and joint_state_publisher_gui
- Kinect, Depth, Ultrasonic and infrared sensors in new Gazebo
- Use of `gz-sim-diff-drive-system` `gz::sim::systems::DiffDrive` for differential drive control.
- Employing xacro files for easier management of robot structure defitnition.
- Launch files for launching Gazebo and Spawning a robot using `ros_gz_sim` and `ros_gz`
- Use of `pointcloud_to_laserscan` for depth camera.
- Examples of `ros_gz_bridge` for bridging ROS2 with Gazebo.
- Modern nav2, slam_toolbox, and rtabmap launch and configuration files.
- Computation and publishing of odometry.
- rviz launch files with proper TF transformation along with different configuration files.
- Use of the new Gazebo (formerly ignition gazebo) 
