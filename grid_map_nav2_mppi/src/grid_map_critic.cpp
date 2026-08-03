#include "grid_map_nav2_mppi/grid_map_critic.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "xtensor/xtensor.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"


namespace mppi::critics
{

void GridMapCritic::initialize()
{
  auto node = parent_.lock().get();

  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(power_, "power", 1);
  getParam(weight_, "weight", 3.81f);
  getParam(enabled_, "enabled", true);
}

void GridMapCritic::score(CriticData & data)
{
  std::lock_guard<std::mutex> lock(map_mutex_);

  auto & trajectories = data.trajectories;
  xt::xtensor<float, 1> costs = xt::zeros<float>({trajectories.x.shape(0)});        // one cost per trajectory

        //For each trajectory, compute the cost based on the grid map values
  for(size_t i = 0; i < trajectories.x.shape(0); ++i) {
    float trajectory_cost = 0.0f;

            //For each pose in each trajectory, get the corresponding grid map value and accumulate the cost
    for(size_t j = 0; j < trajectories.x.shape(1); ++j) {
      double x = trajectories.x(i, j);
      double y = trajectories.y(i, j);

      unsigned int mx, my;
      if (!costmap_->worldToMap(x, y, mx, my)) {continue;}

      unsigned char cost = costmap_->getCost(mx, my);
      if (cost == nav2_costmap_2d::NO_INFORMATION) {continue;}

      trajectory_cost += static_cast<float>(cost) / 254.0f;
    }
    costs(i) = trajectory_cost;
  }
        // Apply weight and add to total trajectory cost
  data.costs += xt::pow(costs * weight_, power_);
}
}// namespace mppi::critics
