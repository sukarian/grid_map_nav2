# grid_map_nav2

A nav2_costmap_2d plugin that incorporates grid_map data (elevation, traversability, slope) into the nav2 ecosystem. The nav2 navigation stack is 2-dimensional, this allows the stack to
let terrain-based costs to inform navigation.

## Features
- Configurable Layer Name
- Configurable Topic
- Nearest Neighbor and Bilinear interpolation methods
- Min/Max value configuration
- Merges with other layers via max-cost

## Dependencies
- ROS 2 (tested on Jazzy)
- nav2_costmap_2d
- grid_map_ros
- grid_map_msgs

## Installation/Build
```
cd ~/ros_ws/src
git clone <your repo>
cd ~/ros_ws
colcon build --packages-select grid_map_nav2
source install/setup.bash
```
