#ifndef GRID_MAP_CRITIC_HPP
#define GRID_MAP_CRITIC_HPP

#include <memory>
#include <string>

#include "nav2_mppi_controller/critic_function.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include <mutex>

namespace mppi::critics
{

class GridMapCritic : public CriticFunction {
public:
  virtual void initialize();
  virtual void score(CriticData & data);

private:
  std::shared_ptr<grid_map::GridMap> latest_map_;
  std::mutex map_mutex_;
  std::string layer_name_;

  float power_;
  float weight_;
  bool enabled_;
};

} // namespace mppi::critics

#endif // GRID_MAP_CRITIC_HPP
