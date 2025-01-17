/**
 * @file utils.cpp
 * @author sjtu
 * @brief Contains common/commonly used funtions and classes
 */


#include <iomanip>  // TODO(vss): replace setw
#include <iostream>
#include <random>

#include "utils/utils.hpp"

// constants
constexpr int spacing_for_grid = 10;

void Node::PrintStatus() const {
  std::cout << "--------------" << '\n'
            << "Node          :" << '\n'
            << "x             : " << x_ << '\n'
            << "y             : " << y_ << '\n'
            << "Cost          : " << cost_ << '\n'
            << "Heuristic cost: " << h_cost_ << '\n'
            << "Id            : " << id_ << '\n'
            << "Pid           : " << pid_ << '\n'
            << "--------------" << '\n';
}

Node Node::operator+(const Node& p) const {
  Node tmp;
  tmp.x_ = this->x_ + p.x_;
  tmp.y_ = this->y_ + p.y_;
  tmp.cost_ = this->cost_ + p.cost_;
  return tmp;
}

Node Node::operator-(const Node& p) const {
  Node tmp;
  tmp.x_ = this->x_ - p.x_;
  tmp.y_ = this->y_ - p.y_;
  return tmp;
}

bool Node::operator==(const Node& p) const {
  return this->x_ == p.x_ && this->y_ == p.y_;
}

bool CompareCoordinates(const Node& p1, const Node& p2) {
  return p1.x_ == p2.x_ && p1.y_ == p2.y_;
}
bool checkOutsideBoundary(const Node& node, const int n) {
  return (node.x_ < 0 || node.y_ < 0
    || node.x_ >= n || node.y_ >= n);
}


bool compare_cost::operator()(const Node& p1, const Node& p2) const {
  // Can modify this to allow tie breaks based on heuristic cost if required
  return p1.cost_ + p1.h_cost_ > p2.cost_ + p2.h_cost_ ||
         (p1.cost_ + p1.h_cost_ == p2.cost_ + p2.h_cost_ &&
          p1.h_cost_ >= p2.h_cost_);
}
bool compare_coordinates::operator()(const Node& p1, const Node& p2)  const {
  return p1.x_ == p2.x_ && p1.y_ == p2.y_;
}

// Possible motions for dijkstra, A*, and similar algorithms.
// Not using this for RRT & RRT* to allow random direction movements.
// TODO(vss): Consider adding option for motion restriction in RRT and RRT* by
//       replacing new node with nearest node that satisfies motion constraints

std::vector<Node> GetMotion() {
  return {
    Node(0, 1, 1, 0, 0, 0),
    Node(1, 0, 1, 0, 0, 0),
    Node(0, -1, 1, 0, 0, 0),
    Node(-1, 0, 1, 0, 0, 0)
    // Node(1, 1, sqrt(2), 0, 0, 0),
    // Node(1, -1, sqrt(2), 0, 0, 0),
    // Node(-1, 1, sqrt(2), 0, 0, 0),
    // Node(-1, -1, sqrt(2), 0, 0, 0)
  };
  // NOTE: Add diagonal movements for A* and D* only after the heuristics in the
  // algorithms have been modified. Refer to README.md. The heuristics currently
  // implemented are based on Manhattan distance and dwill not account for
  // diagonal/ any other motions
}

void MakeGrid(std::vector<std::vector<int>>& grid) {
  int n = grid.size();
  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 eng(rd());  // seed the generator
  std::uniform_int_distribution<int> distr(0, n);  // define the range

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      grid[i][j] = distr(eng) / ((n - 1));  // probability of obstacle is 1/n
      // grid[i][j] = 0; // For no obstacles
    }
  }
}

void PrintPath(const std::vector<Node>& path_vector, const Node& start,
               const Node& goal, std::vector<std::vector<int>>& grid) {
#ifdef CUSTOM_DEBUG_HELPER_FUNCION
  if (path_vector.empty()) {
    std::cout << "No path exists" << '\n';
    PrintGrid(grid);
    return;
  }
  std::cout << "Path (goal to start):" << '\n';
  for (size_t i = 0; i < path_vector.size(); i++) {
    if (CompareCoordinates(goal, path_vector[i])) {
      path_vector[i].PrintStatus();
      grid[path_vector[i].x_][path_vector[i].y_] = 3;
      while (path_vector[i].id_ != start.id_) {
        if (path_vector[i].id_ == path_vector[i].pid_) {
          break;
        }
        for (size_t j = 0; j < path_vector.size(); j++) {
          if (path_vector[i].pid_ == path_vector[j].id_) {
            i = j;
            path_vector[j].PrintStatus();
            grid[path_vector[j].x_][path_vector[j].y_] = 3;
          }
        }
      }
      break;
    }
  }
  grid[start.x_][start.y_] = 3;
  PrintGrid(grid);
#endif  // CUSTOM_DEBUG_HELPER_FUNCION
}

void PrintCost(const std::vector<std::vector<int>>& grid,
               const std::vector<Node>& point_list) {
#ifdef CUSTOM_DEBUG_HELPER_FUNCION
  int n = grid.size();
  std::vector<Node>::const_iterator it_v;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      for (it_v = point_list.begin(); it_v != point_list.end(); ++it_v) {
        if (i == it_v->x_ && j == it_v->y_) {
          std::cout << std::setw(spacing_for_grid) << it_v->cost_ << " , ";
          break;
        }
      }
      if (it_v == point_list.end()) {
        std::cout << std::setw(spacing_for_grid) << "  , ";
      }
    }
    std::cout << '\n' << '\n';
  }
  #endif  // CUSTOM_DEBUG_HELPER_FUNCION
}

void PrintPathInOrder(const std::vector<Node>& path_vector,
                      const Node& /*start*/, const Node& goal,
                      std::vector<std::vector<int>>& grid) {
#ifdef CUSTOM_DEBUG_HELPER_FUNCION
  if (path_vector.empty()) {
    std::cout << "Path not found" << '\n';
    PrintGrid(grid);
    return;
  }
  std::cout << "Path (goal to start):" << '\n';
  size_t i = 0;
  while (!CompareCoordinates(path_vector[i], goal)) {
    i++;
  }
  for (; i > 0; i = i - 1) {
    path_vector[i].PrintStatus();
    grid[path_vector[i].x_][path_vector[i].y_] = 3;
  }
  path_vector[0].PrintStatus();
  grid[path_vector[0].x_][path_vector[0].y_] = 3;
  PrintGrid(grid);
#endif  // CUSTOM_DEBUG_HELPER_FUNCION
}


void LazyPQ::clear() {
  s.clear();
  while(!pq.empty()) {
    pq.pop();
  }
}

void LazyPQ::insert(const NodeKeyPair& t) {
  if(auto p = s.insert(t); !p.second) {
    s.erase(t);
    s.insert(t);
  }
  pq.push(t);
}

void LazyPQ::pop() {
  while(!pq.empty()) {
    if (const auto it = s.find(pq.top()); it == s.end() ||
        (it != s.end() && pq.top().key != it->key)) {
      // Element been removed from set OR
      // Element has been updated in set with new key, and inserted already into pq with new value
      pq.pop();
    } else if (it != s.end() && pq.top().key == it->key) {
      // Found an elelment that is in set and priority queue
      break;
    }
  }
  if (s.empty()) {
    return;
  }
  s.erase(pq.top());
  pq.pop();
  // The loop below allows top() to be const without making the
  // std::priority_queue mutable
  while(!pq.empty()) {
    if (const auto it = s.find(pq.top()); it == s.end() ||
        (it != s.end() && pq.top().key != it->key)) {
      // Element been removed from set OR
      // Element has been updated in set with new key, and inserted already into pq with new value
      pq.pop();
    } else if (it != s.end() && pq.top().key == it->key) {
      // Found an elelment that is in set and priority queue
      break;
    }
  }
}

const NodeKeyPair& LazyPQ::top() const {
  return pq.top();
}

size_t LazyPQ::size() const {
  return s.size();
}

bool LazyPQ::empty() const {
  return s.empty();
}

bool LazyPQ::isElementInStruct(const NodeKeyPair& t) const {
  return s.find(t) != s.end();
}

void LazyPQ::remove(const NodeKeyPair& t) {
  if (s.find(t) != s.end()) {
    s.erase(t);
  }
  // Ensure top() is const
  while(!pq.empty()) {
    if (const auto it = s.find(pq.top()); it == s.end() ||
        (it != s.end() && pq.top().key != it->key)) {
      // Element been removed from set OR
      // Element has been updated in set with new key, and inserted already into pq with new value
      pq.pop();
    } else if (it != s.end() && pq.top().key == it->key) { // Found an elelment that is in set and priority queue
      break;
    }
  }
}
