xterm -e 'roslaunch turtlebot_gazebo turtlebot_world.launch' &
sleep 4
xterm -e 'roslaunch turtlebot_gazebo amcl_demo.launch' &
sleep 2
xterm -e 'roslaunch turtlebot_rviz_launchers view_navigation.launch' &
sleep 1
xterm -e 'rosrun pick_objects pick_objects_node' &
sleep 5
xterm -e 'rosrun add_markers add_markers_node'