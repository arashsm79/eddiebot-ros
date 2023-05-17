# eddiebot_gazebo



## 1. Installation

### 1.1 Dependencies

For running the eddiebot gazebo simulation, following dependencies needs to be installed

- [turtlebot](http://wiki.ros.org/turtlebot)
- [turtlebot-gazebo](http://wiki.ros.org/turtlebot_gazebo)
- [rtabmap-ros](http://wiki.ros.org/rtabmap_ros)

Try following commands to install the dependencies

```bash
sudo apt-get install ros-kinetic-turtlebot ros-kinetic-turtlebot-gazebo ros-kinetic-rtabmap-ros
```

### 1.2 Environment setup

For viewing our self-defined models in the simulation environment, you need to **update** a gazebo global environment variable. Use following commands to update,

```bash
export GAZEBO_MODEL_PATH=<path_to_eddiebot_gazebo>/objects:$GAZEBO_MODEL_PATH
```

For convenience, you can add this command to your `~/.bashrc` or `~/.zshrc`.

## 2. Usage

### 2.1 Simulate with TurtleBot model

Open a terminal, run following command to boot-up gazebo simulation environment,

```bash
roslaunch eddiebot_gazebo turtlebot_world.launch
```

Open a new terminal, run following command to control the robot by using your keyboard

```bash
roslaunch eddiebot_gazebo turtlebot_keyop.launch
```

Open a new terminal, run following command to start `rviz`,

```bash
roslaunch eddiebot_rviz_launchers gazebo_mapping.launch
```

*Note: you can change the simulated world by modifying [turtlebot_world.launch](launch/turtlebot_world.launch) file or using parameter `world_name:=<new_world_name>`* 

### 2.2 SLAM

#### 2.2.1 [Gmapping](http://wiki.ros.org/gmapping)

Once [2.1](#21-simulate-with-turtlebot-model) is done, open a new terminal run following command to launch gmapping

```bash
roslaunch eddiebot_gazebo gmapping_gazebo.launch
```

Once the map is built, use `map_server` to save the explored map (a new terminal is needed)

```bash
rosrun map_server map_saver -f <path_to_map_file>
```

Once the `map_file` is saved to local, we can use acml to navigate the map by using following command

```bash
roslaunch eddiebot_gazebo acml_gazebo.launch map_file:=<path_to_map_file>
```

#### 2.2.2 [Rtabmap](http://wiki.ros.org/rtabmap_ros)

Rtabmap SLAM is an alternative to gmapping. 

Open a new terminal, run following command to launch rtabmap for map building,

```bash
roslaunch eddiebot_gazebo rtabmap_gazebo.launch
```

Once the map is built, run following command for navigating the map

```bash
roslaunch eddiebot_gazebo rtabmap_gazebo.launch localization:=true
```

*Note: by default, map database is store in ~/.ros/rtabmap.db*

For more configuration in rtabmap, please read the file [rtabmap_gazebo.launch](launch/rtabmap_gazebo.launch) 