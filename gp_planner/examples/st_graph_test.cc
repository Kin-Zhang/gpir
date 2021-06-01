/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "common/utils/timer.h"
#include "gp_planner/sdf/signed_distance_field_2d.h"
#include "gp_planner/thirdparty/matplotlib-cpp/matplotlibcpp.h"
#include "gp_planner/thirdparty/traj_min_jerk/traj_min_jerk.hpp"

using namespace planning;
using v2 = Eigen::Vector2d;
namespace plt = matplotlibcpp;

void DFS(const std::vector<std::vector<std::pair<double, double>>>& graph,
         std::vector<Eigen::MatrixXd>* res, int col, Eigen::MatrixXd& route) {
  for (int i = 0; i < graph[col].size(); ++i) {
    route(col) = (graph[col][i].first + graph[col][i].second) / 2.0;
    if (col < graph.size() - 1) {
      DFS(graph, res, col + 1, route);
    } else {
      res->emplace_back(route);
    }
  }
}

int main(int argc, char const* argv[]) {
  OccupancyMap om;
  om.set_cell_number({40, 500});
  om.set_resolution({0.2, 0.2});
  om.set_origin({0.0, 0.0});

  vector_Eigen<v2> obstacle1{v2(2, 50), v2(8, 90), v2(8, 100), v2(2, 60)};
  vector_Eigen<v2> obstacle2{v2(4, 10), v2(8, 20), v2(8, 30), v2(4, 20)};
  om.FillConvexPoly(obstacle1);
  om.FillConvexPoly(obstacle2);
  om.FillEntireRow(0);
  om.FillEntireRow(100);

  TIC;
  SignedDistanceField2D sdf(std::move(om));
  sdf.UpdateSDF();
  TOC("sdf");
  // std::vector<double> t_sample{0, 1, 2, 3, 4, 5, 6, 7, 8};
  // std::vector<std::vector<std::pair<double, double>>> boundaries;
  // om.SearchForVerticalBoundaries(t_sample, &boundaries);

  std::vector<min_jerk::Trajectory> trajs;

  cv::imshow("st_graph", sdf.occupancy_map().BinaryImage());
  cv::imshow("st_graph2", sdf.esdf().ImageSec());
  cv::waitKey(0);

  std::vector<double> obs1_x, obs1_y;
  std::vector<double> obs2_x, obs2_y;
  for (size_t i = 0; i <= obstacle1.size(); ++i) {
    if (i == obstacle1.size()) {
      obs1_x.emplace_back(obstacle1[0].x());
      obs1_y.emplace_back(obstacle1[0].y());
    } else {
      obs1_x.emplace_back(obstacle1[i].x());
      obs1_y.emplace_back(obstacle1[i].y());
    }
  }
  for (size_t i = 0; i <= obstacle2.size(); ++i) {
    if (i == obstacle1.size()) {
      obs2_x.emplace_back(obstacle2[0].x());
      obs2_y.emplace_back(obstacle2[0].y());
    } else {
      obs2_x.emplace_back(obstacle2[i].x());
      obs2_y.emplace_back(obstacle2[i].y());
    }
  }
  double v0 = 5;
  double a_max = 2;
  std::vector<double> t, s_min, s_max;
  for (double i = 0; i < 8; i += 0.1) {
    t.emplace_back(i);
    s_max.emplace_back(v0 * i + 0.5 * a_max * i * i);
    s_min.emplace_back(v0 * i - 0.5 * a_max * i * i);
  }

  plt::plot(obs1_x, obs1_y, "r");
  plt::plot(obs2_x, obs2_y, "r");
  plt::plot(t, s_min, "b");
  plt::plot(t, s_max, "k");
  plt::xlim(0, 9);
  plt::ylim(0, 105);
  plt::show();

  return 0;
}
