#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Lidar callback function
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // Place callback function here
    void publishMessage();
    
    // Update costmap based on laser scan
    void updateCostMap(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    void inflateObstacles();
 
  private:
    robot::CostmapCore costmap_;
    // Place these constructs here
    nav_msgs::msg::OccupancyGrid costmap_grid_;
    int inflation_radius = 3; //meters
    int maximum_cost = 100; // Max cost value for occupied cells/inflated cells
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    
};
 
#endif 