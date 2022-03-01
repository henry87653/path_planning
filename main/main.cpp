/**
 * @file main.cpp
 * @author sjtu
 * @brief Main file where all the algorithms can be used and tested.
 */

#include <iostream>
#include <random>

#include "path_planning/a_star.hpp"

int main() {
  constexpr int n = 21;
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
  PrintGrid(grid);

  // Store points after algorithm has run
  std::vector<std::vector<int>> main_grid = grid;

  // Resetting grid
  // Create object for the algorithm
  // Run algorithm
  // Print the final grid using the path_vector

  // clang-format off
  std::cout << "--------------------------------------------------------" << '\n'
            << "--------------------- ALGORITHM: A* ---------------------" << '\n'
            << "--------------------------------------------------------" << '\n';
  // clang-format on
  grid = main_grid;
  AStar a_star(grid);
  {
    const auto [path_found, path_vector] = a_star.Plan(start, goal);
    PrintPath(path_vector, start, goal, grid);
  }
}
