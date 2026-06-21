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
git clone https://github.com/sukarian/grid_map_nav2.git
cd ~/ros_ws
colcon build --packages-select grid_map_nav2
source install/setup.bash
```
# Usage
```
global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      robot_radius: 0.22
      resolution: 0.05
      track_unknown_space: true
      plugins: ["static_layer", "obstacle_layer", "inflation_layer", "elevation_layer"]
      filters: ["keepout_filter", "speed_filter"]
      elevation_layer:
        plugin: "grid_map_nav2::GridMapLayer"
        topic: "/elevation_mapping/elevation_map"
        layer_name: "traversability"
        min_value: 0.0
        max_value: 1.0
        interpolation_method: "bilinear"
```
# Demo
<img width="1611" height="931" alt="image" src="https://github.com/user-attachments/assets/89b74bf7-acf3-460f-a259-1a4968fb6571" />
Pictured above: large, high-cost region from the elevation mapping layer on top of the tb3 simulation map

# Limitations/Future Work
Currently, this layer is only verified using a static map from a mock publisher. In the future, a dynamic map publishing node will be used to verify functionality.
