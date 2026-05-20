#include "planner_node.hpp"



#include <cmath>
#include <algorithm>
#include <limits>
#include <array>
#include <vector>
#include "geometry_msgs/msg/pose_stamped.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "goal_point",
    10,
    std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1)
  );

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "memory_map",
    10,
    std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1)
  );

  odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom/filtered",
    10,
    std::bind(&PlannerNode::odometryCallback, this, std::placeholders::_1)
  );
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&PlannerNode::planningLoop, this)
  );

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("planned_path", 10);
}
void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received new goal: (%.2f, %.2f)", msg->point.x, msg->point.y);
  waiting_for_goal_ = false;
  goal_x_ = static_cast<float>(msg->point.x);
  goal_y_ = static_cast<float>(msg->point.y);
}
void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received new map: resolution=%.2f, width=%u, height=%u",
    msg->info.resolution, msg->info.width, msg->info.height);
  latest_map_ = *msg; // Store the latest map for planning
}
void PlannerNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = static_cast<float>(msg->pose.pose.position.x);
  robot_y_ = static_cast<float>(msg->pose.pose.position.y);
}
void PlannerNode::planningLoop() {
  if (waiting_for_goal_) {
    RCLCPP_DEBUG(this->get_logger(), "Waiting for goal...");
    return;
  } else if (latest_map_.data.empty()) {
    RCLCPP_WARN(this->get_logger(), "No map received yet, cannot plan.");
    return;
  }
  // Helper values
  const double resolution = latest_map_.info.resolution;
  const double origin_x = latest_map_.info.origin.position.x;
  const double origin_y = latest_map_.info.origin.position.y;
  const int width = static_cast<int>(latest_map_.info.width);
  const int height = static_cast<int>(latest_map_.info.height);

  // map_memory_node treats global map origin as translation only; do the same here
  auto worldToGrid = [&](double wx, double wy, int &gx, int &gy)->bool {
    double fx = (wx - origin_x) / resolution;
    double fy = (wy - origin_y) / resolution;
    gx = static_cast<int>(std::floor(fx));
    gy = static_cast<int>(std::floor(fy));
    if (gx < 0 || gx >= width || gy < 0 || gy >= height) {
      return false;
    }
    return true;
  };

  // grid cell (gx,gy) -> world (wx,wy) at cell center
  auto gridToWorld = [&](int gx, int gy, double &wx, double &wy){
    wx = origin_x + (static_cast<double>(gx) + 0.5) * resolution;
    wy = origin_y + (static_cast<double>(gy) + 0.5) * resolution;
  };

  int robot_grid_x = 0, robot_grid_y = 0;
  if (!worldToGrid(robot_x_, robot_y_, robot_grid_x, robot_grid_y)) {
    RCLCPP_WARN(this->get_logger(), "Robot position out of map bounds: (%.2f, %.2f)", robot_x_, robot_y_);
    return;
  }
  const int robot_index = robot_grid_y * width + robot_grid_x;

  int goal_grid_x = 0, goal_grid_y = 0;
  if (!worldToGrid(goal_x_, goal_y_, goal_grid_x, goal_grid_y)) {
    RCLCPP_WARN(this->get_logger(), "Goal position out of map bounds: (%.2f, %.2f)", goal_x_, goal_y_);
    return;
  }
  const int goal_index = goal_grid_y * width + goal_grid_x;

  RCLCPP_DEBUG(this->get_logger(), "Robot grid=(%d,%d) idx=%d goal grid=(%d,%d) idx=%d", robot_grid_x, robot_grid_y, robot_index, goal_grid_x, goal_grid_y, goal_index);

  // A* planner
  const int size = width * height;
  const double INF = std::numeric_limits<double>::infinity();
  std::vector<double> g_cost(size, INF);
  std::vector<int> parent(size, -1);
  std::vector<char> closed(size, 0);

  while (!min_heap_.empty()) min_heap_.pop();

  g_cost[robot_index] = 0.0;
  min_heap_.push(convert_Index_To_Node(robot_index, 0));

  const std::array<std::pair<int,int>,8> nbrs = {{{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}}};
  const double diag_cost = std::sqrt(2.0);
  bool found = false;

  while (!min_heap_.empty()) {
    GridNode current = min_heap_.top();
    min_heap_.pop();
    int cx = current.x;
    int cy = current.y;
    int cidx = cy * width + cx;
    if (closed[cidx]) continue;
    closed[cidx] = 1;
    if (cidx == goal_index) { found = true; break; }

    for (auto d : nbrs) {
      int nx = cx + d.first;
      int ny = cy + d.second;
      if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
      int nidx = ny * width + nx;
      if (closed[nidx]) continue;
      int8_t occ = latest_map_.data[nidx];
      if (occ >= 70) continue;
      double step_cost = (std::abs(d.first) + std::abs(d.second) == 2) ? diag_cost : 1.0;
      double tentative_g = g_cost[cidx] + step_cost;
      if (tentative_g < g_cost[nidx]) {
        g_cost[nidx] = tentative_g;
        parent[nidx] = cidx;
        GridNode next;
        next.x = nx;
        next.y = ny;
        next.g = g_cost[nidx];
        next.h = std::hypot(static_cast<double>(goal_grid_x - nx), static_cast<double>(goal_grid_y - ny));
        next.f = next.g + next.h;
        min_heap_.push(next);
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(this->get_logger(), "A*: no path found from %d to %d", robot_index, goal_index);
    return;
  }

  // reconstruct path
  std::vector<int> path_idx;
  for (int idx = goal_index; idx != -1; idx = parent[idx]) {
    path_idx.push_back(idx);
  }
  std::reverse(path_idx.begin(), path_idx.end());

  // build nav_msgs::msg::Path
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = latest_map_.header.frame_id.empty() ? std::string("map") : latest_map_.header.frame_id;
  for (int idx : path_idx) {
    int gx = idx % width;
    int gy = idx / width;
    double wx, wy;
    gridToWorld(gx, gy, wx, wy);
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    ps.pose.position.x = wx;
    ps.pose.position.y = wy;
    ps.pose.position.z = 0.0;
    ps.pose.orientation.w = 1.0;
    path_msg.poses.push_back(ps);
  }

  path_pub_->publish(path_msg);

}
GridNode PlannerNode::convert_Index_To_Node(int index, int prev_costs) {
  const int width = static_cast<int>(latest_map_.info.width);
  const int height = static_cast<int>(latest_map_.info.height);
  int x = index % width;
  int y = index / width;
  GridNode n;
  n.x = x;
  n.y = y;
  n.g = static_cast<double>(prev_costs);

  
  double h = 0.0;
  double resolution = latest_map_.info.resolution;
  double origin_x = latest_map_.info.origin.position.x;
  double origin_y = latest_map_.info.origin.position.y;
  int goal_gx = static_cast<int>(std::floor((static_cast<double>(goal_x_) - origin_x) / resolution));
  int goal_gy = static_cast<int>(std::floor((static_cast<double>(goal_y_) - origin_y) / resolution));
  if (goal_gx >= 0 && goal_gx < width && goal_gy >= 0 && goal_gy < height) {
    h = std::hypot(static_cast<double>(goal_gx - x), static_cast<double>(goal_gy - y));
  }
  n.h = h;
  n.f = n.g + n.h;
  return n;

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
