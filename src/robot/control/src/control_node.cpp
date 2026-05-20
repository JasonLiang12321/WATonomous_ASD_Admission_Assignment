#include "control_node.hpp"

#include <algorithm>
#include <limits>

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  // Subscribe to path topic
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "planned_path",
    10,
    std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  );

  // Subscribe to odometry topic
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom/filtered",
    10,
    std::bind(&ControlNode::odometryCallback, this, std::placeholders::_1)
  );

  // Publisher for twist commands
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    "cmd_vel",
    10
  );

  // Timer for control loop (50 Hz)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&ControlNode::controlLoop, this)
  );

  RCLCPP_INFO(this->get_logger(), "Control node initialized");
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received path with %zu poses", msg->poses.size());
  current_path_ = *msg;
  path_received_ = !current_path_.poses.empty();
  last_closest_idx_ = 0;
}

void ControlNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = static_cast<float>(msg->pose.pose.position.x);
  robot_y_ = static_cast<float>(msg->pose.pose.position.y);
  
  // Extract yaw from quaternion: yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;
  robot_yaw_ = static_cast<float>(std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz)));
}

void ControlNode::controlLoop() {
  if (!path_received_ || current_path_.poses.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "No path available, stopping robot");
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  const auto &goal_pose = current_path_.poses.back();
  const double goal_dx = static_cast<double>(goal_pose.pose.position.x) - static_cast<double>(robot_x_);
  const double goal_dy = static_cast<double>(goal_pose.pose.position.y) - static_cast<double>(robot_y_);
  const double goal_distance = std::sqrt(goal_dx * goal_dx + goal_dy * goal_dy);

  if (goal_distance <= goal_tolerance_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Goal reached, stopping robot");
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  // Pure Pursuit Algorithm
  // 1. Find closest point on path
  size_t closest_idx = findClosestPathPoint(robot_x_, robot_y_);
  
  // 2. Find lookahead point
  double lookahead_x, lookahead_y;
  if (!findLookaheadPoint(closest_idx, robot_x_, robot_y_, lookahead_x, lookahead_y)) {
    RCLCPP_WARN(this->get_logger(), "Could not find lookahead point");
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return;
  }

  // 3. Calculate pure pursuit curvature and convert to angular velocity
  double steering_angle = calculateSteeringAngle(robot_x_, robot_y_, robot_yaw_, 
                                                  lookahead_x, lookahead_y);

  // 4. Clamp angular velocity
  double angular_velocity = std::max(-max_angular_velocity_, 
                                     std::min(max_angular_velocity_, steering_angle));

  // 5. Publish twist command
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = linear_velocity_;
  cmd.angular.z = angular_velocity;
  cmd_vel_pub_->publish(cmd);

  RCLCPP_DEBUG(this->get_logger(), 
    "Robot: (%.2f, %.2f, %.2f) | Lookahead: (%.2f, %.2f) | Steering: %.3f rad/s",
    robot_x_, robot_y_, robot_yaw_, lookahead_x, lookahead_y, angular_velocity);
}

size_t ControlNode::findClosestPathPoint(double robot_x, double robot_y) {
  if (current_path_.poses.empty()) return 0;

  size_t closest_idx = last_closest_idx_;
  double min_dist_sq = std::numeric_limits<double>::infinity();

  // Search forward and backward from last index
  const size_t search_window = std::min(size_t(10), current_path_.poses.size());
  size_t search_start = (last_closest_idx_ > search_window) ? last_closest_idx_ - search_window : 0;
  size_t search_end = std::min(last_closest_idx_ + search_window, current_path_.poses.size());

  for (size_t i = search_start; i < search_end; ++i) {
    double px = current_path_.poses[i].pose.position.x;
    double py = current_path_.poses[i].pose.position.y;
    double dx = px - robot_x;
    double dy = py - robot_y;
    double dist_sq = dx*dx + dy*dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      closest_idx = i;
    }
  }

  last_closest_idx_ = closest_idx;
  return closest_idx;
}

bool ControlNode::findLookaheadPoint(size_t closest_idx, double robot_x, double robot_y,
                                      double &lookahead_x, double &lookahead_y) {
  if (current_path_.poses.empty()) return false;

  // Start from closest point and search forward along path
  double accumulated_dist = 0.0;
  for (size_t i = closest_idx; i < current_path_.poses.size() - 1; ++i) {
    double x0 = current_path_.poses[i].pose.position.x;
    double y0 = current_path_.poses[i].pose.position.y;
    double x1 = current_path_.poses[i+1].pose.position.x;
    double y1 = current_path_.poses[i+1].pose.position.y;
    double segment_dist = std::sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
    
    if (accumulated_dist + segment_dist >= lookahead_distance_) {
      // Lookahead point is on this segment
      double t = (lookahead_distance_ - accumulated_dist) / segment_dist;
      t = std::max(0.0, std::min(1.0, t)); // Clamp to [0, 1]
      lookahead_x = x0 + t * (x1 - x0);
      lookahead_y = y0 + t * (y1 - y0);
      return true;
    }
    accumulated_dist += segment_dist;
  }

  // If we reach here, use last point in path
  if (!current_path_.poses.empty()) {
    lookahead_x = current_path_.poses.back().pose.position.x;
    lookahead_y = current_path_.poses.back().pose.position.y;
    return true;
  }

  return false;
}

double ControlNode::calculateSteeringAngle(double robot_x, double robot_y, double robot_yaw,
                                            double lookahead_x, double lookahead_y) {
  // Pure pursuit curvature law
  double dx = lookahead_x - robot_x;
  double dy = lookahead_y - robot_y;
  double heading_to_point = std::atan2(dy, dx);
  double heading_error = heading_to_point - robot_yaw;
  // Normalize heading error to [-pi, pi]
  while (heading_error > M_PI) heading_error -= 2.0 * M_PI;
  while (heading_error < -M_PI) heading_error += 2.0 * M_PI;

  const double curvature = (2.0 * std::sin(heading_error)) / std::max(lookahead_distance_, 1e-6);
  return linear_velocity_ * curvature;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
