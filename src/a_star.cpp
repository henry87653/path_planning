/**
 * @file a_star.cpp
 * @author sjtu
 * @brief Contains the AStar class
 */

#include <cmath>
#include <queue>
#include <unordered_set>
#include <vector>

#ifdef BUILD_INDIVIDUAL
#include <random>
#endif  // BUILD_INDIVIDUAL

#include "path_planning/a_star.hpp"

std::tuple<bool, std::vector<Node>> AStar::Plan(const Node& start,
                                                const Node& goal) {
  grid_ = original_grid_;
  std::priority_queue<Node, std::vector<Node>, compare_cost> open_list;
  std::unordered_set<Node, NodeIdAsHash, compare_coordinates> closed_list;

  const std::vector<Node> motion = GetMotion();
  open_list.push(start);

  // Main loop
  while (!open_list.empty()) {
    Node current = open_list.top();
    open_list.pop();
    current.id_ = current.x_ * n_ + current.y_;
    if (CompareCoordinates(current, goal)) {
      closed_list.insert(current);
      grid_[current.x_][current.y_] = 2;
      return {true, ConvertClosedListToPath(closed_list, start, goal)};
    }
    grid_[current.x_][current.y_] = 2;  // Point opened
    for (const auto& m : motion) {
      Node new_point = current + m;
      new_point.id_ = n_ * new_point.x_ + new_point.y_;
      new_point.pid_ = current.id_;
      new_point.h_cost_ =
          std::abs(new_point.x_ - goal.x_) + std::abs(new_point.y_ - goal.y_);
      if (CompareCoordinates(new_point, goal)) {
        open_list.push(new_point);
        break;
      }
      if (checkOutsideBoundary(new_point, n_)) {
        continue;  // Check boundaries
      }
      if (grid_[new_point.x_][new_point.y_] != 0) {
        continue;  // obstacle or visited
      }
      open_list.push(new_point);
    }
    closed_list.insert(current);
  }
  return {false, {}};
}

std::vector<Node> AStar::ConvertClosedListToPath(
    std::unordered_set<Node, NodeIdAsHash, compare_coordinates>& closed_list,
    const Node& start, const Node& goal) {
  auto current = *closed_list.find(goal);
  std::vector<Node> path;
  while (!CompareCoordinates(current, start)) {
    path.push_back(current);
    if (const auto it = closed_list.find(
            Node(current.pid_ / n_, current.pid_ % n_, 0, 0, current.pid_));
        it != closed_list.end()) {
      current = *it;
    } else {
      std::cout << "Error in calculating path \n";
      return {};
    }
  }
  path.push_back(start);
  return path;
}

#ifdef BUILD_INDIVIDUAL
/**
 * @brief Script main function. Generates start and end nodes as well as grid,
 * then creates the algorithm object and calls the main algorithm function.
 * @return 0
 */
int main() {
  constexpr int n = 11;
  std::vector<std::vector<int>> grid(n, std::vector<int>(n, 0));
  MakeGrid(grid);

  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, n - 1);  // define the range

  Node start(distr(eng), distr(eng), 0, 0, 0, 0);
  Node goal(distr(eng), distr(eng), 0, 0, 0, 0);

  start.id_ = start.x_ * n + start.y_;
  start.pid_ = start.x_ * n + start.y_;
  goal.id_ = goal.x_ * n + goal.y_;
  start.h_cost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

  // Make sure start and goal are not obstacles and their ids are correctly
  // assigned.
  grid[start.x_][start.y_] = 0;
  grid[goal.x_][goal.y_] = 0;

  start.PrintStatus();
  goal.PrintStatus();

  PrintGrid(grid);

  AStar new_a_star(grid);
  const auto [path_found, path_vector] = new_a_star.Plan(start, goal);

  PrintPath(path_vector, start, goal, grid);
  return 0;
}
#endif  // BUILD_INDIVIDUAL
