#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "planner_core.hpp"

#include <queue>
#include <vector>
#include <functional>
#include <cmath>
#include <limits>
#include <algorithm>
#include <array>

struct GridNode {
  int x;
  int y;

  double g; // cost from start
  double h; // heuristic
  double f; // total cost

  bool operator>(const GridNode& other) const {
    return f > other.f;
  }
};

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void planningLoop();

  private:
  GridNode convert_Index_To_Node(int index, int prev_costs);
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  nav_msgs::msg::OccupancyGrid latest_map_;
  std::string map_frame_id_ = "map";
  bool waiting_for_goal_ = true;
  float goal_x_ = 0.0;
  float goal_y_ = 0.0;
  float robot_x_ = 0.0;
  float robot_y_ = 0.0;
  rclcpp::TimerBase::SharedPtr timer_;
  robot::PlannerCore planner_;
  std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> min_heap_;
};

#endif 
