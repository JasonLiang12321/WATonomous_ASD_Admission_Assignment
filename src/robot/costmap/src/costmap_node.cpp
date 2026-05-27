#include <chrono>
#include <memory>
#include <cmath>
#include "costmap_node.hpp"
 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Initialize the constructs and their parameters
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  // Publisher for the OccupancyGrid so visualizers (Foxglove, RViz2) can display the map
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", rclcpp::QoS(1).transient_local());
  // Initialize occupancy grid once so the data buffer exists before callbacks
  costmap_grid_.header.frame_id = "map";
  costmap_grid_.info.resolution = 0.1; // meters per cell
  costmap_grid_.info.width = 300;
  costmap_grid_.info.height = 300;
  costmap_grid_.info.origin.position.x = -15.0;
  costmap_grid_.info.origin.position.y = -15.0;
  costmap_grid_.info.origin.orientation.w = 1.0;
  costmap_grid_.data.assign(static_cast<size_t>(costmap_grid_.info.width) * static_cast<size_t>(costmap_grid_.info.height), -1);
  
  // Subscribe to the /lidar topic to receive LaserScan messages
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Costmap node initialized and listening to /lidar topic");
  

}
 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  // auto message = std_msgs::msg::String();
  // message.data = "Hello, ROS 2!";
  // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  // string_pub_->publish(message);
}

// Lidar callback function to process LaserScan messages
void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // Extract LaserScan message fields:
  // - angle_min: Starting angle of the scan
  // - angle_max: Ending angle of the scan
  // - angle_increment: Angular distance between measurements (radians)
  // - ranges: Array of distance measurements (meters)
  costmap_grid_.data.assign(static_cast<size_t>(costmap_grid_.info.width) * static_cast<size_t>(costmap_grid_.info.height), 0);
  costmap_grid_.header.frame_id = msg->header.frame_id.empty() ? std::string("robot/chassis/lidar") : msg->header.frame_id;
  
  RCLCPP_DEBUG(this->get_logger(), 
    "Received LaserScan: angle_min=%.2f, angle_max=%.2f, angle_increment=%.4f, ranges_size=%zu",
    msg->angle_min, msg->angle_max, msg->angle_increment, msg->ranges.size());
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float angle = msg->angle_min + i * msg->angle_increment;
      float range = msg->ranges[i];
      if (range > msg->range_min && range < msg->range_max) {
        int x_grid = static_cast<int>((range * cos(angle)) / costmap_grid_.info.resolution) + static_cast<int>(costmap_grid_.info.width) / 2;
        int y_grid = static_cast<int>((range * sin(angle)) / costmap_grid_.info.resolution) + static_cast<int>(costmap_grid_.info.height) / 2;
        // check bounds before writing
        if (x_grid >= 0 && x_grid < static_cast<int>(costmap_grid_.info.width) &&
            y_grid >= 0 && y_grid < static_cast<int>(costmap_grid_.info.height)) {
          costmap_grid_.data[static_cast<size_t>(y_grid) * costmap_grid_.info.width + static_cast<size_t>(x_grid)] = 100; // Mark as occupied
          RCLCPP_DEBUG(this->get_logger(), "Updated costmap at grid (%d, %d) with range %.2f", x_grid, y_grid, range);
          RCLCPP_DEBUG(this->get_logger(), "Set costmap value at index %zu to %d", static_cast<size_t>(y_grid) * costmap_grid_.info.width + static_cast<size_t>(x_grid), costmap_grid_.data[static_cast<size_t>(y_grid) * costmap_grid_.info.width + static_cast<size_t>(x_grid)]);
        }
      }
    }
  inflateObstacles();
  updateCostMap(msg);
  // TODO: Process laser scan data to update costmap
  // This is where you'll convert range measurements into a costmap representation
}
void CostmapNode::inflateObstacles() {
  for (size_t y = 0; y < costmap_grid_.info.height; ++y) {
    for (size_t x = 0; x < costmap_grid_.info.width; ++x) {
      if (costmap_grid_.data[static_cast<size_t>(y) * costmap_grid_.info.width + static_cast<size_t>(x)] == 100) { // Occupied cell
        for (int dy = -inflation_radius; dy <= inflation_radius; ++dy) {
          for (int dx = -inflation_radius; dx <= inflation_radius; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx >= 0 && nx < static_cast<int>(costmap_grid_.info.width) && 
                ny >= 0 && ny < static_cast<int>(costmap_grid_.info.height)) {
              float distance = std::sqrt(dx * dx + dy * dy) * costmap_grid_.info.resolution;
              if (distance <= inflation_radius) {
                size_t index = static_cast<size_t>(ny) * costmap_grid_.info.width + static_cast<size_t>(nx);
                int inflated_cost = static_cast<int>(maximum_cost * (1 - (distance / static_cast<float>(inflation_radius))));
                costmap_grid_.data[index] = static_cast<signed char>(std::max(
                  static_cast<int>(costmap_grid_.data[index]), 
                 inflated_cost));
                RCLCPP_DEBUG(this->get_logger(), "Maximum cost: %d, Distance: %.2f, Inflation radius: %d, Inflation cost: %d", maximum_cost, distance, inflation_radius, inflated_cost);
                RCLCPP_DEBUG(this->get_logger(), "Set costmap value at index %zu to %d", index, costmap_grid_.data[index]);
              }
            }
          }
        }
      }
    }
  }
}

void CostmapNode::updateCostMap(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

  // Stamp and ensure data buffer size
    costmap_grid_.header.stamp = msg->header.stamp;
    
    map_pub_->publish(costmap_grid_);

    RCLCPP_DEBUG(this->get_logger(), "Published costmap %ux%u", costmap_grid_.info.width, costmap_grid_.info.height);
  


}

 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}