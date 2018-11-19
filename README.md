# ping pong robot

### Simulación con kinect y sin errores (laser 120º)
Lanzamos:
* roslaunch turtlebot_gazebo turtlebot_world.launch
* roslaunch ping_pong_robot ping_pong_amcl.launch
* roslaunch turtlebot_rviz_launchers view_navigation.launch --screen

### Simulación con rplidar  y con errores
Lanzamos:
* roslaunch ping_pong_robot ping_pong_world.launch
* roslaunch ping_pong_robot ping_pong_amcl.launch
* roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
