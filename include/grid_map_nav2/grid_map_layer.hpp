#ifndef GRID_MAP_LAYER_HPP_
#define GRID_MAP_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"

namespace grid_map_nav2
{

class GridMapLayer : public nav2_costmap_2d::Layer
{
public:
  GridMapLayer();

  virtual void onInitialize();
  virtual void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y,
    double * max_x,
    double * max_y);
  virtual void updateCosts(
      nav2_costmap_2d::Costmap2D & master_grid,
      int min_i, int min_j, int max_i, int max_j);

  virtual void reset()
  {
    return;
  }

  virtual void onFootprintChanged();

  virtual void gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);

  virtual bool isClearable() {return false;}

private:
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

  bool need_recalculation_;
  std::string layer_name_;
  std::string topic_;
  double min_value_;
  double max_value_;
  
  std::mutex map_mutex_;
  grid_map_msgs::msg::GridMap::SharedPtr latest_map_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr subscription_;
  


};

}  // namespace grid_map_nav2_core

#endif  // GRID_MAP_LAYER_HPP_