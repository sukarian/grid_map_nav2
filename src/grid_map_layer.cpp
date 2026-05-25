#include "grid_map_nav2/grid_map_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"
#include "grid_map_msgs/msg/grid_map.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "pluginlib/class_list_macros.hpp"

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

std::mutex map_mutex_;
grid_map_msgs::msg::GridMap::SharedPtr latest_map_;

namespace grid_map_nav2
{
    GridMapLayer::GridMapLayer()
    :   last_min_x_(-std::numeric_limits<float>::max()),
        last_min_y_(-std::numeric_limits<float>::max()),
        last_max_x_(std::numeric_limits<float>::max()),
        last_max_y_(std::numeric_limits<float>::max())
    {
    }

    void GridMapLayer::onInitialize()
    {
        auto node = node_.lock(); 

        declareParameter("layer_name", rclcpp::ParameterValue("traversability"));
        declareParameter("topic", rclcpp::ParameterValue("/elevation_mapping/elevation_map"));
        declareParameter("min_value", rclcpp::ParameterValue(0.0));
        declareParameter("max_value", rclcpp::ParameterValue(1.0));

        node->get_parameter(name_ + "." + "layer_name", layer_name_);
        node->get_parameter(name_ + "." + "topic", topic_);
        node->get_parameter(name_ + "." + "min_value", min_value_);
        node->get_parameter(name_ + "." + "max_value", max_value_);

        subscription_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
            topic_,
            rclcpp::QoS(1).reliable(),
            std::bind(&GridMapLayer::gridMapCallback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(rclcpp::get_logger("grid_map_layer"),
            "Params: layer=%s topic=%s min=%.2f max=%.2f",
            layer_name_.c_str(), topic_.c_str(), min_value_, max_value_);

        need_recalculation_ = true;
    }

    void 
    GridMapLayer::gridMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        latest_map_ = msg;
    }

    void GridMapLayer::onFootprintChanged()
    {
        // No action needed when footprint changes
    }

    void GridMapLayer::updateBounds(
        double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
        double* min_x, double* min_y, double* max_x, double* max_y)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!latest_map_) { return; }

        double map_x = latest_map_->info.pose.position.x;
        double map_y = latest_map_->info.pose.position.y;
        double half_x = latest_map_->info.length_x / 2.0;
        double half_y = latest_map_->info.length_y / 2.0;

        *min_x = std::min(*min_x, map_x - half_x);
        *min_y = std::min(*min_y, map_y - half_y);
        *max_x = std::max(*max_x, map_x + half_x);
        *max_y = std::max(*max_y, map_y + half_y);
    }

    void GridMapLayer::updateCosts(
        nav2_costmap_2d::Costmap2D & master_grid,
        int min_i, int min_j, int max_i, int max_j)
    {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (!latest_map_) { return; }

        // Find the layer index directly from the message
        int layer_idx = -1;
        for (size_t i = 0; i < latest_map_->layers.size(); i++) {
            if (latest_map_->layers[i] == layer_name_) {
                layer_idx = static_cast<int>(i);
                break;
            }
        }
        if (layer_idx < 0) {
            RCLCPP_WARN(rclcpp::get_logger("grid_map_layer"),
                "Layer '%s' not found", layer_name_.c_str());
            return;
        }

        double resolution = latest_map_->info.resolution;
        if (resolution <= 0.0) { return; }

        int cols = static_cast<int>(std::round(latest_map_->info.length_x / resolution));
        int rows = static_cast<int>(std::round(latest_map_->info.length_y / resolution));
        double map_x = latest_map_->info.pose.position.x;
        double map_y = latest_map_->info.pose.position.y;

        const auto & data = latest_map_->data[layer_idx].data;

        if (static_cast<int>(data.size()) != cols * rows) {
            RCLCPP_WARN(rclcpp::get_logger("grid_map_layer"),
                "Data size mismatch: expected %d got %zu", cols * rows, data.size());
            return;
        }

        float range = max_value_ - min_value_;
        if (std::abs(range) < 1e-6f) { return; }

        for (int col = 0; col < cols; col++) {
            for (int row = 0; row < rows; row++) {
                float value = data[col * rows + row];

                double wx = map_x + (col - cols / 2.0 + 0.5) * resolution;
                double wy = map_y + (row - rows / 2.0 + 0.5) * resolution;

                unsigned int mx, my;
                if (!master_grid.worldToMap(wx, wy, mx, my)) { continue; }

                if (std::isnan(value)) {
                    master_grid.setCost(mx, my, nav2_costmap_2d::NO_INFORMATION);
                    continue;
                }

                float normalized = (value - min_value_) / range;
                unsigned char cost = static_cast<unsigned char>(normalized * 254.0f);
                master_grid.setCost(mx, my, cost);
            }
        }
    }
}
PLUGINLIB_EXPORT_CLASS(grid_map_nav2::GridMapLayer, nav2_costmap_2d::Layer)
