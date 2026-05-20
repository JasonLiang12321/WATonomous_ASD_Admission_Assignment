#include "map_memory_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace
{

std::string renderMemoryMap(const nav_msgs::msg::OccupancyGrid & memory_map)
{
  constexpr size_t kDisplayWidth = 30;
  constexpr size_t kDisplayHeight = 30;

  std::ostringstream output;
  output << "Memory map " << memory_map.info.width << "x" << memory_map.info.height
         << " (coarse view " << kDisplayWidth << "x" << kDisplayHeight << ")\n";

  if (memory_map.data.empty() || memory_map.info.width == 0 || memory_map.info.height == 0) {
    output << "<empty>";
    return output.str();
  }

  const size_t width_step = std::max<size_t>(1, memory_map.info.width / kDisplayWidth);
  const size_t height_step = std::max<size_t>(1, memory_map.info.height / kDisplayHeight);

  for (size_t display_row = 0; display_row < kDisplayHeight; ++display_row) {
    const size_t row_start = display_row * height_step;
    const size_t row_end = std::min(static_cast<size_t>(memory_map.info.height), row_start + height_step);

    for (size_t display_col = 0; display_col < kDisplayWidth; ++display_col) {
      const size_t col_start = display_col * width_step;
      const size_t col_end = std::min(static_cast<size_t>(memory_map.info.width), col_start + width_step);

      int max_value = -1;
      for (size_t row = row_start; row < row_end; ++row) {
        for (size_t col = col_start; col < col_end; ++col) {
          const int8_t cell_value = memory_map.data[row * memory_map.info.width + col];
          max_value = std::max(max_value, static_cast<int>(cell_value));
        }
      }

      char cell_char = ' ';
      if (max_value >= 90) {
        cell_char = '#';
      } else if (max_value >= 50) {
        cell_char = '*';
      } else if (max_value > 0) {
        cell_char = '.';
      }

      output << cell_char;
    }

    output << '\n';
  }

  return output.str();
}

}  // namespace

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  // Subscribe to the /odometry topic to receive Odometry messages
  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odometryCallback, this, std::placeholders::_1));

  // Subscribe to the /costmap topic to receive OccupancyGrid messages
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::mapCallback, this, std::placeholders::_1));

  // Publisher for the memory map
  memory_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/memory_map", rclcpp::QoS(1).transient_local());

  // Timer to periodically publish the memory map
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MapMemoryNode::publishAndCalcMemory, this));

  memory_map_.header.frame_id = "map";
  memory_map_.info.resolution = 0.1; // meters per cell
  memory_map_.info.width = 300;
  memory_map_.info.height = 300;
  memory_map_.info.origin.position.x = -15.0;
  memory_map_.info.origin.position.y = -15.0;
  memory_map_.info.origin.orientation.w = 1.0;
  memory_map_.data.assign(static_cast<size_t>(memory_map_.info.width) * static_cast<size_t>(memory_map_.info.height), 0);
}

void MapMemoryNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Process odometry data and update map memory
  RCLCPP_DEBUG(this->get_logger(), "Received Odometry: position=(%.2f, %.2f)",
    msg->pose.pose.position.x, msg->pose.pose.position.y);
  robot_x_ = static_cast<float>(msg->pose.pose.position.x);
  robot_y_ = static_cast<float>(msg->pose.pose.position.y);
  robot_orientation_[0] = static_cast<float>(msg->pose.pose.orientation.x);
  robot_orientation_[1] = static_cast<float>(msg->pose.pose.orientation.y);
  robot_orientation_[2] = static_cast<float>(msg->pose.pose.orientation.z);
  robot_orientation_[3] = static_cast<float>(msg->pose.pose.orientation.w);

  robot_yaw_ = 2.0 * std::atan2(
    static_cast<double>(robot_orientation_[2]),
    static_cast<double>(robot_orientation_[3]));
}

void MapMemoryNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Process incoming map data
  RCLCPP_DEBUG(this->get_logger(), "Received OccupancyGrid: resolution=%.2f, width=%u, height=%u",
    msg->info.resolution, msg->info.width, msg->info.height);
  // Example: copy incoming map into memory_map_ (simple overwrite)
  latest_map_ = *msg; // Store the latest map for potential use in memory updates
}

void MapMemoryNode::publishAndCalcMemory() {
  if (latest_map_.data.empty() || latest_map_.info.width == 0 || latest_map_.info.height == 0) {
    memory_map_.header.stamp = this->now();
    memory_map_pub_->publish(memory_map_);
    RCLCPP_DEBUG(this->get_logger(), "Published memory map without updates");
    return;
  }

  nav_msgs::msg::OccupancyGrid merged_map = memory_map_;
  const double cos_yaw = std::cos(robot_yaw_);
  const double sin_yaw = std::sin(robot_yaw_);
  const double local_width = static_cast<double>(latest_map_.info.width) * latest_map_.info.resolution;
  const double local_height = static_cast<double>(latest_map_.info.height) * latest_map_.info.resolution;
  const double local_origin_x = -0.5 * local_width;
  const double local_origin_y = -0.5 * local_height;

  for (size_t row = 0; row < latest_map_.info.height; ++row) {
    for (size_t col = 0; col < latest_map_.info.width; ++col) {
      const size_t local_index = row * latest_map_.info.width + col;
      const int8_t new_value = latest_map_.data[local_index];

      if (new_value < 0) {
        continue;
      }

      const double local_x = local_origin_x + (static_cast<double>(col) + 0.5) * latest_map_.info.resolution;
      const double local_y = local_origin_y + (static_cast<double>(row) + 0.5) * latest_map_.info.resolution;

      const double world_x = static_cast<double>(robot_x_) + (cos_yaw * local_x - sin_yaw * local_y);
      const double world_y = static_cast<double>(robot_y_) + (sin_yaw * local_x + cos_yaw * local_y);

      const int global_col = static_cast<int>(std::floor(
        (world_x - memory_map_.info.origin.position.x) / memory_map_.info.resolution));
      const int global_row = static_cast<int>(std::floor(
        (world_y - memory_map_.info.origin.position.y) / memory_map_.info.resolution));

      if (global_col < 0 || global_row < 0 ||
        global_col >= static_cast<int>(memory_map_.info.width) ||
        global_row >= static_cast<int>(memory_map_.info.height)) {
        continue;
      }

      const size_t global_index = static_cast<size_t>(global_row) * memory_map_.info.width +
        static_cast<size_t>(global_col);

      if (new_value > merged_map.data[global_index]) {
        merged_map.data[global_index] = new_value;
      }
    }
  }

  // Stamp and publish the current state of the memory map
  merged_map.header.stamp = this->now();
  memory_map_ = merged_map;
  memory_map_pub_->publish(memory_map_);
  RCLCPP_DEBUG(this->get_logger(), "\n%s", renderMemoryMap(memory_map_).c_str());


  RCLCPP_DEBUG(this->get_logger(), "Published merged memory map");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
