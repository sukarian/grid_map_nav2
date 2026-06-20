#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "grid_map_nav2/grid_map_layer.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

TEST(GridMapLayerTest, UpdateCosts_GridMapHighRes_Bilinear)
{
    // Create a GridMapLayer instance
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");

    auto clock = node->get_clock();

    auto tf_buffer =
        std::make_shared<tf2_ros::Buffer>(clock);

    auto layered_costmap =
        std::make_shared<nav2_costmap_2d::LayeredCostmap>(
            "map",
            false,
            false);

    auto layer =
        std::make_shared<grid_map_nav2::GridMapLayer>();

    layer->initialize(
        layered_costmap.get(),
        "grid_map_layer",
        tf_buffer.get(),
        node,
        nullptr);

    // Create a mock GridMap message
    auto grid_map_msg = std::make_shared<grid_map_msgs::msg::GridMap>();
    grid_map_msg->info.resolution = 1.0;
    grid_map_msg->info.length_x = 5.0;
    grid_map_msg->info.length_y = 5.0;
    grid_map_msg->info.pose.position.x = 0.0;
    grid_map_msg->info.pose.position.y = 0.0;
    grid_map_msg->basic_layers.push_back("traversability");
    grid_map_msg->header.frame_id = "map";
    grid_map_msg->layers.push_back("traversability");
    std_msgs::msg::Float32MultiArray layer_data;
    layer_data.data = {
        0.0, 0.5, 1.0, 0.5, 0.0,
        0.5, 1.0, 1.0, 1.0, 0.5,
        1.0, 1.0, 1.0, 1.0, 1.0,
        0.5, 1.0, 1.0, 1.0, 0.5,
        0.0, 0.5, 1.0, 0.5, 0.0
    };
    grid_map_msg->data.push_back(layer_data);

    // Simulate receiving the GridMap message
    layer->gridMapCallback(grid_map_msg);

    // Create a Costmap2D instance
    nav2_costmap_2d::Costmap2D costmap(5, 5, 1.0, -2.5, -2.5, 0.0);

    // Update costs using the GridMapLayer
    layer->updateCosts(costmap, 0, 0, costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    std::cout << "Costmap after update:" << std::endl;
    for (unsigned int j = 0; j < costmap.getSizeInCellsY(); j++) {
        for (unsigned int i = 0; i < costmap.getSizeInCellsX(); i++) {
            std::cout << static_cast<int>(costmap.getCost(i, j)) << " ";
        }        std::cout << std::endl;
    }

    // Verify that the costs have been updated correctly
    // GridMap center value 1.0 → cost 254 at costmap(0,0)
    ASSERT_EQ(costmap.getCost(0, 0), 0);

    // GridMap value 0.5 cells adjacent to overlap boundary
    ASSERT_EQ(costmap.getCost(1, 0), 127);
    ASSERT_EQ(costmap.getCost(0, 1), 127);

    // GridMap corner value 0.0 → cost 0 at costmap(2,2)
    ASSERT_EQ(costmap.getCost(1, 1), 254);

    // Outside GridMap extent → unwritten → default 0
    ASSERT_EQ(costmap.getCost(5, 5), 0);
}

TEST(GridMapLayerTest, UpdateCosts_GridMapLowRes_Bilinear)
{
    // Create a GridMapLayer instance
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");

    auto clock = node->get_clock();

    auto tf_buffer =
        std::make_shared<tf2_ros::Buffer>(clock);

    auto layered_costmap =
        std::make_shared<nav2_costmap_2d::LayeredCostmap>(
            "map",
            false,
            false);

    auto layer =
        std::make_shared<grid_map_nav2::GridMapLayer>();

    layer->initialize(
        layered_costmap.get(),
        "grid_map_layer",
        tf_buffer.get(),
        node,
        nullptr);

    // Create a mock GridMap message
    auto grid_map_msg = std::make_shared<grid_map_msgs::msg::GridMap>();
    grid_map_msg->info.resolution = 1.0;
    grid_map_msg->info.length_x = 5.0;
    grid_map_msg->info.length_y = 5.0;
    grid_map_msg->info.pose.position.x = 0.0;
    grid_map_msg->info.pose.position.y = 0.0;
    grid_map_msg->basic_layers.push_back("traversability");
    grid_map_msg->header.frame_id = "map";
    grid_map_msg->layers.push_back("traversability");

    std_msgs::msg::Float32MultiArray layer_data;
    layer_data.data = {
        0.0, 0.5, 1.0, 0.5, 0.0,
        0.5, 1.0, 1.0, 1.0, 0.5,
        1.0, 1.0, 1.0, 1.0, 1.0,
        0.5, 1.0, 1.0, 1.0, 0.5,
        0.0, 0.5, 1.0, 0.5, 0.0
    };
    grid_map_msg->data.push_back(layer_data);

    // Simulate receiving the GridMap message
    layer->gridMapCallback(grid_map_msg);

    // Create a Costmap2D instance
    nav2_costmap_2d::Costmap2D costmap(10, 10, 0.5, -2.5, -2.5, 0.0);

    // Update costs using the GridMapLayer
    layer->updateCosts(costmap, 0, 0, costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    std::cout << "Costmap after update:" << std::endl;
    for (unsigned int j = 0; j < costmap.getSizeInCellsY(); j++) {
        for (unsigned int i = 0; i < costmap.getSizeInCellsX(); i++) {
            std::cout << static_cast<int>(costmap.getCost(i, j)) << " ";
        }        std::cout << std::endl;
    }

    // Verify that the costs have been updated correctly
    // GridMap center value 1.0 → cost 254 at costmap(0,0)
    ASSERT_EQ(costmap.getCost(0, 0), 0);
    ASSERT_EQ(costmap.getCost(1, 1), 63);
    ASSERT_EQ(costmap.getCost(1, 2), 127);
    ASSERT_EQ(costmap.getCost(2, 1), 127);
    ASSERT_EQ(costmap.getCost(10, 10), 0);
}

TEST(GridMapLayerTest, UpdateCosts_GridMapHighRes_Nearest)
{
    // Create a GridMapLayer instance
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");

    auto clock = node->get_clock();

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("grid_map_layer.interpolation_method", "nearest_neighbor")
    });


    auto tf_buffer =
        std::make_shared<tf2_ros::Buffer>(clock);

    auto layered_costmap =
        std::make_shared<nav2_costmap_2d::LayeredCostmap>(
            "map",
            false,
            false);

    auto layer =
        std::make_shared<grid_map_nav2::GridMapLayer>();
    
    layer->initialize(
        layered_costmap.get(),
        "grid_map_layer",
        tf_buffer.get(),
        node,
        nullptr);

    // Create a mock GridMap message
    auto grid_map_msg = std::make_shared<grid_map_msgs::msg::GridMap>();
    grid_map_msg->info.resolution = 1.0;
    grid_map_msg->info.length_x = 5.0;
    grid_map_msg->info.length_y = 5.0;
    grid_map_msg->info.pose.position.x = 0.0;
    grid_map_msg->info.pose.position.y = 0.0;
    grid_map_msg->basic_layers.push_back("traversability");
    grid_map_msg->header.frame_id = "map";
    grid_map_msg->layers.push_back("traversability");
    
    std_msgs::msg::Float32MultiArray layer_data;
    layer_data.data = {
        0.0, 0.5, 1.0, 0.5, 0.0,
        0.5, 1.0, 1.0, 1.0, 0.5,
        1.0, 1.0, 1.0, 1.0, 1.0,
        0.5, 1.0, 1.0, 1.0, 0.5,
        0.0, 0.5, 1.0, 0.5, 0.0
    };
    grid_map_msg->data.push_back(layer_data);

    // Simulate receiving the GridMap message
    layer->gridMapCallback(grid_map_msg);

    // Create a Costmap2D instance
    nav2_costmap_2d::Costmap2D costmap(5, 5, 1.0, -2.5, -2.5, 0.0);

    // Update costs using the GridMapLayer
    layer->updateCosts(costmap, 0, 0, costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    std::cout << "Costmap after update:" << std::endl;
    for (unsigned int j = 0; j < costmap.getSizeInCellsY(); j++) {
        for (unsigned int i = 0; i < costmap.getSizeInCellsX(); i++) {
            std::cout << static_cast<int>(costmap.getCost(i, j)) << " ";
        }        std::cout << std::endl;
    }

    // Verify that the costs have been updated correctly
    // GridMap center value 1.0 → cost 254 at costmap(0,0)
    ASSERT_EQ(costmap.getCost(0, 0), 0);

    // GridMap value 0.5 cells adjacent to overlap boundary
    ASSERT_EQ(costmap.getCost(1, 0), 127);
    ASSERT_EQ(costmap.getCost(0, 1), 127);

    // GridMap corner value 0.0 → cost 0 at costmap(2,2)
    ASSERT_EQ(costmap.getCost(1, 1), 254);

    // Outside GridMap extent → unwritten → default 0
    ASSERT_EQ(costmap.getCost(5, 5), 0);
}

TEST(GridMapLayerTest, UpdateCosts_GridMapLowRes_Nearest)
{
    // Create a GridMapLayer instance
    auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");

    auto clock = node->get_clock();

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("grid_map_layer.interpolation_method", "nearest_neighbor")
    });

    auto tf_buffer =
        std::make_shared<tf2_ros::Buffer>(clock);

    auto layered_costmap =
        std::make_shared<nav2_costmap_2d::LayeredCostmap>(
            "map",
            false,
            false);

    auto layer =
        std::make_shared<grid_map_nav2::GridMapLayer>();

    layer->initialize(
        layered_costmap.get(),
        "grid_map_layer",
        tf_buffer.get(),
        node,
        nullptr);

    // Create a mock GridMap message
    auto grid_map_msg = std::make_shared<grid_map_msgs::msg::GridMap>();
    grid_map_msg->info.resolution = 1.0;
    grid_map_msg->info.length_x = 5.0;
    grid_map_msg->info.length_y = 5.0;
    grid_map_msg->info.pose.position.x = 0.0;
    grid_map_msg->info.pose.position.y = 0.0;
    grid_map_msg->basic_layers.push_back("traversability");
    grid_map_msg->header.frame_id = "map";
    grid_map_msg->layers.push_back("traversability");
    std_msgs::msg::Float32MultiArray layer_data;
    layer_data.data = {
        0.0, 0.5, 1.0, 0.5, 0.0,
        0.5, 1.0, 1.0, 1.0, 0.5,
        1.0, 1.0, 1.0, 1.0, 1.0,
        0.5, 1.0, 1.0, 1.0, 0.5,
        0.0, 0.5, 1.0, 0.5, 0.0
    };
    grid_map_msg->data.push_back(layer_data);

    // Simulate receiving the GridMap message
    layer->gridMapCallback(grid_map_msg);

    // Create a Costmap2D instance
    nav2_costmap_2d::Costmap2D costmap(10, 10, 0.5, -2.5, -2.5, 0.0);

    // Update costs using the GridMapLayer
    layer->updateCosts(costmap, 0, 0, costmap.getSizeInCellsX(), costmap.getSizeInCellsY());

    std::cout << "Costmap after update:" << std::endl;
    for (unsigned int j = 0; j < costmap.getSizeInCellsY(); j++) {
        for (unsigned int i = 0; i < costmap.getSizeInCellsX(); i++) {
            std::cout << static_cast<int>(costmap.getCost(i, j)) << " ";
        }        std::cout << std::endl;
    }

    // Verify that the costs have been updated correctly
    // GridMap center value 1.0 → cost 254 at costmap(0,0)
    ASSERT_EQ(costmap.getCost(0, 0), 0);
    ASSERT_EQ(costmap.getCost(1, 1), 63);
    ASSERT_EQ(costmap.getCost(1, 2), 127);
    ASSERT_EQ(costmap.getCost(2, 1), 127);
    ASSERT_EQ(costmap.getCost(10, 10), 0);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);

    auto result = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return result;
}