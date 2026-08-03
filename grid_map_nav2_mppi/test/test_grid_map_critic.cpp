#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_mppi_controller/tools/parameters_handler.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/path.hpp"
#include "nav2_mppi_controller/motion_models.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "grid_map_nav2_mppi/grid_map_critic.hpp"
#include "nav2_util/lifecycle_node.hpp"

using namespace mppi;
using namespace mppi::critics;

TEST(GridMapCriticTest, HighCostTerrainPenalized)
{
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("my_node");
    auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
        "dummy_costmap", "", "dummy_costmap", true);    std::string name = "test";
    ParametersHandler param_handler(node);
    rclcpp_lifecycle::State lstate;
    costmap_ros->on_configure(lstate);

    models::State state;
    models::ControlSequence control_sequence;
    models::Trajectories generated_trajectories;
    generated_trajectories.reset(2, 10);  // 2 trajectories, 10 time steps
    models::Path path;
    geometry_msgs::msg::Pose goal;
    path.reset(10);
    xt::xtensor<float, 1> costs = xt::zeros<float>({2});  // one cost per trajectory
    float model_dt = 0.1;

    CriticData data =
  {state, generated_trajectories, path, goal, costs, model_dt,
    false, nullptr, nullptr, std::nullopt, std::nullopt};

    data.motion_model = std::make_shared<DiffDriveMotionModel>();


    // Set up costmap with high cost on right side
    auto * costmap = costmap_ros->getCostmap();
    unsigned int size_x = costmap->getSizeInCellsX();
    unsigned int size_y = costmap->getSizeInCellsY();
    for (unsigned int mx = size_x / 2; mx < size_x; mx++) {
    for (unsigned int my = 0; my < size_y; my++) {
      costmap->setCost(mx, my, 254);
    }
    }



    // Trajectory 0: stays on left (low cost) side
    for (size_t j = 0; j < 10; j++) {
    generated_trajectories.x(0, j) = 1.0f;
    generated_trajectories.y(0, j) = static_cast<float>(j) * 0.1f;
    }

    // Trajectory 1: stays on right (high cost) side
    for (size_t j = 0; j < 10; j++) {
    generated_trajectories.x(1, j) = 4.0f;
    generated_trajectories.y(1, j) = static_cast<float>(j) * 0.1f;
    }

    // Initialize critic
    GridMapCritic critic;
    critic.on_configure(node, "mppi", "GridMapCritic", costmap_ros, &param_handler);
    RCLCPP_INFO(rclcpp::get_logger("test"),
    "Costmap origin=(%.2f, %.2f) size=%dx%d res=%.2f",
    costmap->getOriginX(), costmap->getOriginY(),
    costmap->getSizeInCellsX(), costmap->getSizeInCellsY(),
    costmap->getResolution());

    critic.score(data);


    // Trajectory through high cost terrain should cost more
    EXPECT_GT(costs(1), costs(0));
    EXPECT_NEAR(costs(0), 0.0f, 1e-3);  // left side is free
    EXPECT_GT(costs(1), 0.0f);           // right side has cost
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
