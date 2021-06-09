/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "common/utils/timer.h"
#include "gp_planner/gp/gp_path_optimizer.h"

using namespace planning;

int main(int argc, char *argv[]) {
  google::InstallFailureSignalHandler();
  google::InitGoogleLogging(argv[0]);
  google::SetStderrLogging(google::INFO);
  FLAGS_colorlogtostderr = true;

  OccupancyMap occupancy_map;
  occupancy_map.set_origin({0, -5});
  occupancy_map.set_cell_number(std::array<int, 2>{900, 100});
  occupancy_map.set_resolution({0.1, 0.1});
  occupancy_map.FillCircle(Eigen::Vector2d(30, -1), 0.2);
  // occupancy_map.FillCircle(Eigen::Vector2d(50, 2), 0.2);
  occupancy_map.FillCircle(Eigen::Vector2d(70, 1), 0.2);
  // occupancy_map.FillCircle(Eigen::Vector2d(70, 3), 0.2);
  auto sdf = std::make_shared<SignedDistanceField2D>(std::move(occupancy_map));
  sdf->UpdateSDF();

  std::vector<double> hint{30, 70};

  TIC;
  GPPathOptimizer gp_path_optimizer(sdf);

  std::vector<double> lb, ub;
  std::vector<std::vector<std::pair<double, double>>> boundaries;
  sdf->mutable_occupancy_map()->SearchForVerticalBoundaries(hint, &boundaries);

  Eigen::Vector2d init(0, 1);
  if (!gp_path_optimizer.DecideInitialPathBoundary(init, 0.5, hint, boundaries,
                                                   &lb, &ub)) {
    LOG(ERROR) << "test failed";
  } else {
    LOG(INFO) << "test succeed";
  }
  TOC("decider");

  vector_Eigen2d lb_line{init}, mb_line{init}, ub_line{init};
  for (int i = 0; i < boundaries.size(); ++i) {
    lb_line.emplace_back(hint[i], lb[i]);
    mb_line.emplace_back(hint[i], 0.5 * (lb[i] + ub[i]));
    ub_line.emplace_back(hint[i], ub[i]);
  }
  sdf->mutable_occupancy_map()->PolyLine(lb_line);
  sdf->mutable_occupancy_map()->PolyLine(ub_line);
  sdf->mutable_occupancy_map()->PolyLine(mb_line);
  cv::imshow("123", sdf->occupancy_map().BinaryImage());
  cv::waitKey(0);

  return 0;
}
