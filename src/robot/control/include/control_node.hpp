#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <vector>
#include <cmath>

#include "control_core.hpp"

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlLoop();

    // Pure pursuit helpers
    size_t findClosestPathPoint(double robot_x, double robot_y);
    bool findLookaheadPoint(size_t closest_idx, double robot_x, double robot_y, 
                            double &lookahead_x, double &lookahead_y);
    double calculateSteeringAngle(double robot_x, double robot_y, double robot_yaw,
                                   double lookahead_x, double lookahead_y);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path current_path_;
    float robot_x_ = 0.0;
    float robot_y_ = 0.0;
    float robot_yaw_ = 0.0;

    const double lookahead_distance_ = 1;  // meters
    const double linear_velocity_ = 1.2;      // m/s
    const double max_angular_velocity_ = 2.0; // rad/s
    const double goal_tolerance_ = 0.25;      // meters
    bool path_received_ = false;
    size_t last_closest_idx_ = 0;

    robot::ControlCore control_;
};

#endif
