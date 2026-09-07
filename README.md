# grid_map_nav2
 
Nav2 integration for the [grid_map](https://github.com/ANYbotics/grid# grid_map_nav2
 
Nav2 integration for the [grid_map](https://github.com/ANYbotics/grid_map) library. This metapackage provides a Nav2 costmap layer plugin and an MPPI critic that allow terrain-aware cost data from grid_map to inform robot navigation.
 
## Motivation
 
Nav2's built-in costmap system is fundamentally 2D and geometry-based — it marks cells as occupied or free based on sensor observations, but has no concept of terrain type or mechanical properties. The [grid_map](https://github.com/ANYbotics/grid_map) library from ANYbotics a multi-layer 2d costmap format that scores cells based on roughness, traversability, slope, and elevation.
 
This package bridges nav2 and grid_map. It was developed in response to [this open issue](https://github.com/ANYbotics/elevation_mapping/issues/272) in the `elevation_mapping` repository, which is no longer actively maintained. 

## Package Structure
 
```
grid_map_nav2/
├── package.xml                    # metapackage
├── grid_map_nav2_costmap/         # Nav2 costmap layer plugin
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── grid_map_layer.xml         # pluginlib registration
│   ├── include/
│   │   └── grid_map_nav2/
│   │       └── grid_map_layer.hpp
│   ├── src/
│   │   └── grid_map_layer.cpp
│   └── test/
│       └── test_grid_map_layer.cpp
└── grid_map_nav2_mppi/            # Nav2 MPPI controller critic
    ├── CMakeLists.txt
    ├── package.xml
    ├── grid_map_critic.xml        # pluginlib registration
    ├── include/
    │   └── grid_map_nav2_mppi/
    │       └── grid_map_critic.hpp
    ├── src/
    │   └── grid_map_critic.cpp
    └── test/
        └── test_grid_map_critic.cpp
```
 
## Architecture
 
This integration lives as a standalone metapackage rather than being merged into either `grid_map` or Nav2 directly. This is deliberate, as it doesn't add dependencies to grid_map or nav2.
 
### Costmap Layer (`grid_map_nav2_costmap`)
 
Implements `nav2_costmap_2d::Layer`. Subscribes to a `grid_map_msgs/msg/GridMap` topic, reads a configurable layer (e.g. `traversability`, `slope`, `roughness`), normalizes values to Nav2's cost range (0–254), and writes costs into the master costmap grid. Composites correctly with other layers by taking the maximum of its cost and any existing cost at each cell, so lethal obstacles from the obstacle layer are never erased.
 
Supports two interpolation modes for handling resolution mismatches between the GridMap and the costmap:
 
- **Bilinear** (default) — smooth interpolation between four surrounding GridMap cells, appropriate for continuous terrain data like elevation or slope
- **Nearest neighbor** — faster, preserves sharp cost boundaries, appropriate for discrete traversability masks
When the costmap resolution is coarser than the GridMap resolution, the plugin samples all GridMap cells within each costmap cell's footprint and takes the maximum value, ensuring that a single impassable GridMap cell is not diluted by surrounding traversable ones.
 
### MPPI Critic (`grid_map_nav2_mppi`)
 
Implements `mppi::critics::CriticFunction`. Reads directly from the composite costmap provided by the base class (`costmap_`), which already includes contributions from the costmap layer above. For each sampled MPPI trajectory, accumulates cost along the full trajectory path by querying costmap cells at each pose. Trajectories that traverse high-cost terrain receive higher penalty scores, steering MPPI toward terrain-friendly paths even when they are geometrically longer.
 
The critic and costmap layer are **complementary**:
 
- The costmap layer influences the **global planner's route** by making high-cost terrain visible to path planning
- The MPPI critic fine-tunes the **local controller's trajectory selection** within that route, penalizing sustained traversal of rough terrain
## Dependencies
 
- ROS 2 Jazzy
- `nav2_costmap_2d`
- `nav2_mppi_controller`
- `grid_map_ros`
- `grid_map_msgs`
- `pluginlib`
- `rclcpp`
## Installation
 
```bash
cd ~/ros_ws/src
git clone https://github.com/<your-username>/grid_map_nav2
cd ~/ros_ws
colcon build --packages-select grid_map_nav2_costmap grid_map_nav2_mppi
source install/setup.bash
```
 
Install Nav2 binaries if not already present:
 
```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-mppi-controller
```
 
## Usage
 
Add the plugins to your `nav2_params.yaml`. The costmap layer and MPPI critic are configured independently.
 
### Costmap Layer
 
```yaml
local_costmap:
  local_costmap:
    ros__parameters:
      plugins: ["obstacle_layer", "inflation_layer", "elevation_layer"]
      elevation_layer:
        plugin: "grid_map_nav2::GridMapLayer"
        topic: "/elevation_mapping/elevation_map"
        layer_name: "traversability"
        min_value: 0.0
        max_value: 1.0
        interpolation_method: "bilinear"  # or "nearest_neighbor"
```
 
Place `elevation_layer` last in the plugins list so its `max` compositing correctly preserves lethal costs written by earlier layers.
 
Multiple instances of the plugin can be loaded simultaneously with different configurations — for example, one consuming a `traversability` layer and another consuming a `slope` layer:
 
```yaml
plugins: ["obstacle_layer", "inflation_layer", "traversability_layer", "slope_layer"]
traversability_layer:
  plugin: "grid_map_nav2::GridMapLayer"
  topic: "/elevation_mapping/elevation_map"
  layer_name: "traversability"
  min_value: 0.0
  max_value: 1.0
slope_layer:
  plugin: "grid_map_nav2::GridMapLayer"
  topic: "/elevation_mapping/elevation_map"
  layer_name: "slope"
  min_value: 0.0
  max_value: 0.5
```
 
### MPPI Critic
 
```yaml
controller_server:
  ros__parameters:
    FollowPath:
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign",
                "PathAlign", "PathDist", "GoalDist", "GridMapCritic"]
      GridMapCritic:
        plugin: "grid_map_nav2_mppi::GridMapCritic"
        enabled: true
        cost_weight: 5.0
        power: 1
```
 
## Parameters
 
### `grid_map_nav2_costmap::GridMapLayer`
 
| Parameter | Type | Default | Description |
|---|---|---|---|
| `topic` | string | `/elevation_mapping/elevation_map` | GridMap topic to subscribe to |
| `layer_name` | string | `traversability` | Which GridMap layer to read costs from |
| `min_value` | double | `0.0` | GridMap value that maps to cost 0 (free) |
| `max_value` | double | `1.0` | GridMap value that maps to cost 254 (lethal) |
| `interpolation_method` | string | `bilinear` | `bilinear` or `nearest_neighbor` for fine costmap resolution |
 
**Value convention:** the plugin assumes the ETH/ANYbotics convention where `1.0 = fully traversable` and `0.0 = impassable`. Values are inverted during normalization so that high traversability maps to low cost. If your GridMap layer uses the opposite convention, swap `min_value` and `max_value`.
 
### `grid_map_nav2_mppi::GridMapCritic`
 
| Parameter | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `true` | Enable or disable the critic |
| `cost_weight` | float | `3.81` | Multiplier applied to trajectory cost |
| `power` | int | `1` | Exponent applied to cost — higher values penalize bad terrain superlinearly |
 
## Testing
 
```bash
colcon test --packages-select grid_map_nav2_costmap grid_map_nav2_mppi --event-handlers console_direct+
colcon test-result --verbose
```
 
The costmap layer test suite covers:
 
- Correct cost normalization with same-resolution GridMap and costmap (bilinear and nearest neighbor)
- Correct bilinear interpolation when costmap resolution is finer than GridMap resolution
- Correct nearest-neighbor sampling in the same case
- NaN cell handling (maps to `NO_INFORMATION`)
- Cells outside GridMap bounds are left unwritten
The MPPI critic test covers:
 
- Trajectories through high-cost terrain accumulate more cost than trajectories through free space
- Zero cost for trajectories entirely in free space
## Known Limitations
 
- Only tested with a mock GridMap publisher and in TurtleBot3 Gazebo simulation. 
- The coarse-resolution averaging path (when costmap resolution is coarser than GridMap resolution) uses the maximum value within each costmap cell's footprint. Mean aggregation is not currently supported.

## Future Work
 
- Global costmap support with a globally-registered elevation map
- Mean aggregation option for the coarse-resolution path
- Uncertainty-aware cost propagation using elevation map variance
- Integration with online terrain parameter estimation (Bekker-Wong soil mechanics) for mechanically-informed traversability cost
## License
 
Apache 2.0

