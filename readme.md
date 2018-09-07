# The home service robot

![Screenshot of Home service robot costmap]

Checkout the [YouTube Video] of the project reaching to the designated pickup and drop off zones.

# Step 1. Add Official ROS packages
- SLAM_gmapping
- Turtlebot
- Turtlebot_navigation
- Turtlebot_simulator
- Turtlebot_interactions

# Step 2. Custom U World
- Simple U model and save it in a Gazebo world.
- Add simple U world for SLAM.

# Step 3. Wall_follower node
- Made a wall_follower package
- Add `C++` code in `src/wall_follower.cpp`
- Edited CMakeLists.txt. Add rule for compiling  wall_follower node

# Step 4. Mapping the Simple U world
- reduced the `linearUpdate` and `angularUpdate`
- increased the `minimunScore` to a large value.
This prevented the jumps from the robot for large feature poor environments.
- Save the map file in World directory.

# Step 5. Navigation 

- AMCL: Given the right path for the map file in World to the map server for AMCL.

# Step 6. Pick objects Package

- Create a `pick_objects` package for commanding the robot to go to pickup and drop-off zones.
- Add `C++` code in the `src/pick_objects.cpp` 
- Edited CMakeLists.txt. Add rule for compiling  pick_objects node

# Step 7. Add markers
- Create a `add_markers` package for commanding the robot to go to pickup and drop-off zones.
- Add `C++` code in the `src/add_markers.cpp` to display the visual markers in RViz.
- Edited CMakeLists.txt. Add rule for compiling  add_markers node.

# Step 8. Adding virtual object
- Published virtual object when the robot reaches the pickup and drop point.

To implement this, a new parameter `zone` was set. 

- Setting the parameter server when reaching pickup and drop zone.

```cpp
ros::NodeHandle n;
n.setParam("zone", "pick");
```

The value of the `zone` parameter was checked in the add_markers node and when in a zone, the marker was published in that zone.


[Screenshot of Home service robot costmap]: imgs/home_service.png
[YouTube Video]: https://youtu.be/BCUyZKllxmw