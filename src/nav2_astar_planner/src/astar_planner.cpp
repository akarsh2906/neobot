#include <cmath>
#include <queue>
#include <unordered_map>
#include <vector>
#include "nav2_astar_planner/astar_planner.hpp"

namespace nav2_astar_planner
{

void AStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
}

void AStarPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "Cleaning up plugin %s", name_.c_str());
}

void AStarPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "Activating plugin %s", name_.c_str());
}

void AStarPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "Deactivating plugin %s", name_.c_str());
}

bool AStarPlanner::isWithinBounds(int x, int y)
{
  return x >= 0 && x < costmap_->getSizeInCellsX() && y >= 0 && y < costmap_->getSizeInCellsY();
}

float AStarPlanner::getHeuristic(int x1, int y1, int x2, int y2)
{
  return std::hypot(x2 - x1, y2 - y1);  // Euclidean distance as heuristic
}

std::vector<std::pair<int, int>> AStarPlanner::getNeighbors(int x, int y)
{
  // Return valid neighbors (8-connected grid)
  std::vector<std::pair<int, int>> neighbors;
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) continue;
      int nx = x + dx, ny = y + dy;
      if (isWithinBounds(nx, ny)) {
        neighbors.emplace_back(nx, ny);
      }
    }
  }
  return neighbors;
}

nav_msgs::msg::Path AStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path global_path;

  // Verify start and goal are in the global frame
  if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(node_->get_logger(), "Start or goal is not in the global frame");
    return global_path;
  }

  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // Convert start and goal to grid coordinates
  unsigned int start_x, start_y, goal_x, goal_y;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);

  // Priority queue for A* search
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_list;
  std::unordered_map<int, std::pair<int, int>> came_from;
  std::unordered_map<int, float> cost_so_far;

  // Add start node to the open list
  open_list.push({static_cast<int>(start_x), static_cast<int>(start_y), 0, getHeuristic(start_x, start_y, goal_x, goal_y)});
  cost_so_far[start_y * costmap_->getSizeInCellsX() + start_x] = 0;

  while (!open_list.empty()) {
    auto current = open_list.top();
    open_list.pop();

    // Check if we've reached the goal
    if (current.x == static_cast<int>(goal_x) && current.y == static_cast<int>(goal_y)) {
      // Reconstruct path
      int x = goal_x, y = goal_y;
      while (!(x == start_x && y == start_y)) {
        geometry_msgs::msg::PoseStamped pose;
        costmap_->mapToWorld(x, y, pose.pose.position.x, pose.pose.position.y);
        pose.pose.orientation.w = 1.0;
        pose.header.frame_id = global_frame_;
        global_path.poses.insert(global_path.poses.begin(), pose);

        auto previous = came_from[y * costmap_->getSizeInCellsX() + x];
        x = previous.first;
        y = previous.second;
      }
      return global_path;
    }

    // Explore neighbors
    for (auto &[nx, ny] : getNeighbors(current.x, current.y)) {
      if (costmap_->getCost(nx, ny) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) continue;

      float new_cost = cost_so_far[current.y * costmap_->getSizeInCellsX() + current.x] + 1.0;  // Constant cost
      int neighbor_index = ny * costmap_->getSizeInCellsX() + nx;

      if (cost_so_far.find(neighbor_index) == cost_so_far.end() || new_cost < cost_so_far[neighbor_index]) {
        cost_so_far[neighbor_index] = new_cost;
        float priority = new_cost + getHeuristic(nx, ny, goal_x, goal_y);
        open_list.push({nx, ny, new_cost, priority});
        came_from[neighbor_index] = {current.x, current.y};
      }
    }
  }

  RCLCPP_ERROR(node_->get_logger(), "Failed to find a path to the goal");
  return global_path;
}

}  // namespace nav2_astar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_astar_planner::AStarPlanner, nav2_core::GlobalPlanner)
