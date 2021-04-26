#include "dijkstra.hpp"

using namespace common;

int main(int argc, char const* argv[]) {
  Dijkstra<int> ds;
  ds.AddEdge(0, 1, 15);
  ds.AddEdge(0, 2, 10);
  ds.AddEdge(1, 3, 4);
  ds.AddEdge(2, 3, 3);
  std::vector<int> path;
  ds.FindPath(0, 3, &path);
  for (const auto& p : path) {
    std::cout << p << std::endl;
  }
  return 0;
}
