# eddiebot_rviz_launchers

RVIZ launcher for eddiebot. This package provides a collection of rivz configurations for viewing / testing the eddiebot.



Here is a brief introduction to every launch file:

- [view_model.launch](launch/view_model.launch) is used to view the robot model defined by the urdf files, check [HERE](../../eddiebot/eddiebot_description) to see the usage.
- [view_mapping.launch](launch/view_mapping.launch) is used to view the robot mapping information when the eddiebot performing navigation/localization functions, check [HERE](../../eddiebot_apps/eddiebot_navigation) to see the usage.
- [gazebo_mapping.launch](launch/gazebo_mapping.launch) is used for viewing the information of eddiebot gazebo simulation task. The usage could be found [HERE](../../eddiebot_simulator/eddiebot_gazebo#2-usage).
