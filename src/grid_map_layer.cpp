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
        declareParameter("interpolation_method", rclcpp::ParameterValue("bilinear"));

        node->get_parameter(name_ + "." + "layer_name", layer_name_);
        node->get_parameter(name_ + "." + "topic", topic_);
        node->get_parameter(name_ + "." + "min_value", min_value_);
        node->get_parameter(name_ + "." + "max_value", max_value_);
        node->get_parameter(name_ + "." + "interpolation_method", interpolation_method_);

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

        for (int mx = 0; mx < master_grid.getSizeInCellsX(); mx++) {
            for (int my = 0; my < master_grid.getSizeInCellsY(); my++) {
                double wx, wy;
                master_grid.mapToWorld(mx, my, wx, wy);
                float value  = std::numeric_limits<float>::lowest();;

                if(master_grid.getResolution() <= resolution){

                    // Map world position back to GridMap index
                    double col_d = cols / 2.0 - (wx - map_x) / resolution - 0.5;
                    double row_d = rows / 2.0 - (wy - map_y) / resolution - 0.5;

                    // Skip cells outside GridMap bounds
                    if (col_d < 0 || col_d > cols - 1 || row_d < 0 || row_d > rows - 1) continue;

                    
                    if(interpolation_method_ == "nearest_neighbor"){
                        double col = static_cast<unsigned int>(std::round(col_d));
                        double row = static_cast<unsigned int>(std::round(row_d));
                        col = std::clamp(col, 0.0, static_cast<double>(cols - 1));
                        row = std::clamp(row, 0.0, static_cast<double>(rows - 1));
                        value = data[row * cols + col];
                    }
                    else{
                        // Bilinear interpolation
                        int col0 = static_cast<int>(std::floor(col_d));
                        int row0 = static_cast<int>(std::floor(row_d));
                        int col1 = col0 + 1;
                        int row1 = row0 + 1;

                        double dx = col_d - col0;
                        double dy = row_d - row0;

                        // Clamp indices to valid range
                        col0 = std::clamp(col0, 0, cols - 1);
                        row0 = std::clamp(row0, 0, rows - 1);
                        col1 = std::clamp(col1, 0, cols - 1);
                        row1 = std::clamp(row1, 0, rows - 1);

                        float v00 = data[row0 * cols + col0];
                        float v10 = data[row0 * cols + col1];
                        float v01 = data[row1 * cols + col0];
                        float v11 = data[row1 * cols + col1];

                        value = (v00 * (1 - dx) * (1 - dy)) +
                                    (v10 * dx * (1 - dy)) +
                                    (v01 * (1 - dx) * dy) +
                                    (v11 * dx * dy);
                    }
                }
                else{
                    double half = master_grid.getResolution() / 2.0;
                    for(double sx = wx - half; sx < wx + half; sx += resolution){
                        for(double sy = wy - half; sy < wy + half; sy += resolution){
                            // Map world position back to GridMap index
                            double col_d = cols / 2.0 - (sx - map_x) / resolution - 0.5;
                            double row_d = rows / 2.0 - (sy - map_y) / resolution - 0.5;

                            // Skip cells outside GridMap bounds
                            if (col_d < 0 || col_d > cols - 1 || row_d < 0 || row_d > rows - 1) continue;
                            int col = static_cast<int>(std::round(col_d));
                            int row = static_cast<int>(std::round(row_d));
                            col = std::clamp(col, 0, cols - 1);
                            row = std::clamp(row, 0, rows - 1);
                            float val = data[row * cols + col];
                            value = std::max(value, val);
                        }
                    }
                }
                float normalized = std::clamp(static_cast<float>(value - min_value_) / range, 0.0f, 1.0f);
                unsigned char new_cost = static_cast<unsigned char>((normalized) * 254.0f);

                // Composite with existing cost — highest cost wins
                unsigned char existing = master_grid.getCost(mx, my);
                master_grid.setCost(mx, my, std::max<unsigned char>(existing, new_cost));            
            }
        }
    }
}  // namespace grid_map_nav2
PLUGINLIB_EXPORT_CLASS(grid_map_nav2::GridMapLayer, nav2_costmap_2d::Layer)
