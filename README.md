# Eddiebot ROS
> ROS2 packages for Parallax Eddie robot along with simulations using Gazebo (formerly ignition gazebo)

* [eddiebot_bringup](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_bringup): The driver for the Eddie robot board. It establishes a connection to the board using UART and acts as a bridge between ROS2 and Eddie commands that the firmware on the board understands. For more details about the commands please checkout the documentation directory.
* [eddiebot_description](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_description): This package defines structure of the robot using URDF modelsf for every component of the robot. These models are used for generating kinematic and tf information. It uses the `diff_drive` plugin for differential drive control of the robot. The launch files start `robot_state_publisher` that reads the URDF model and generates static TF transforms, and `joint_state_publisher` that sends out dynamic TF transforms for joint position updates.
* [eddiebot_gazebo](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_gazebo): Contains some sample worlds to test the robot in, along with a bunch of launch files that are able to setup Gazebo, spawn Eddie into the world, and necessary bridges for ROS2 and Gazebo communication.
* [eddiebot_nav](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_nav): Contains the launch files for the setup and launch of the navigation stack using nav2, slam_toolbox, depthimage_to_laserscan, and various other packages.
* [eddiebot_msgs](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_msgs): Definition of specific topics and services for Eddie robot.
* [eddiebot_odom](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_odom): Publishes odometry information useful for mapping and localization. It publishes the transform between the frames `/odom` and `/base_footprint`.
* [eddiebot_rviz](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_rviz): Rviz configurations and launch files for visualizing and monitoring various aspects of Eddie.
* [eddiebot_vel_controller](https://github.com/arashsm79/eddiebot-ros/tree/main/eddiebot_vel_controller): A converter between different velocity types.



