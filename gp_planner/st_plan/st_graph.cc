/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "gp_planner/st_plan/st_graph.h"

#include <glog/logging.h>

#include <random>

#include "common/utils/timer.h"
#include "gp_planner/thirdparty/matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

namespace planning {

void StGraph::BuildStGraph(const std::vector<Obstacle>& dynamic_obstacles,
                           const GPPath& gp_path) {
  common::Timer timer;

  int size = dynamic_obstacles.size();
  st_block_segments_.resize(size);

  for (int i = 0; i < size; ++i) {
    GetObstacleBlockSegment(dynamic_obstacles[i], gp_path,
                            &st_block_segments_[i]);
  }
  timer.End("projection step");
  timer.Reset();

  OccupancyMap grid_map;
  grid_map.set_origin({0.0, init_s_[0]});
  grid_map.set_resolution({0.1, 0.1});
  grid_map.set_cell_number({82, 1000});

  vector_Eigen<Eigen::Vector2d> corners;
  corners.resize(4);

  for (const auto& st_block_segment : st_block_segments_) {
    if (st_block_segment.empty()) continue;
    for (const auto& st_points : st_block_segment) {
      if (st_points.empty()) continue;
      if (st_points.size() == 1) {
        corners.clear();
        corners[0] = Eigen::Vector2d(st_points[0].t, st_points[0].s_l);
        corners[1] = Eigen::Vector2d(st_points[0].t, st_points[0].s_u);
        corners[2] = Eigen::Vector2d(st_points[0].t + 0.1, st_points[0].s_u);
        corners[3] = Eigen::Vector2d(st_points[0].t + 0.1, st_points[0].s_l);
        grid_map.FillConvexPoly(corners);
        continue;
      }
      for (size_t i = 0; i < st_points.size() - 1; ++i) {
        corners.clear();
        corners.emplace_back(st_points[i].t, st_points[i].s_l);
        corners.emplace_back(st_points[i].t, st_points[i].s_u);
        corners.emplace_back(st_points[i + 1].t, st_points[i + 1].s_u);
        corners.emplace_back(st_points[i + 1].t, st_points[i + 1].s_l);
        grid_map.FillConvexPoly(corners);
      }
    }
  }

  timer.End("fill step");
  timer.Reset();
  sdf_ = std::make_unique<SignedDistanceField2D>(std::move(grid_map));
  sdf_->UpdateVerticalSDF();
  timer.End("sdf update");

  cv::imshow("st", sdf_->occupancy_map().BinaryImage());
  cv::imshow("st_sdf", sdf_->esdf().ImageSec());
  cv::waitKey(0);
}

void StGraph::GetObstacleBlockSegment(
    const Obstacle& obstacle, const GPPath& gp_path,
    std::vector<std::vector<StPoint>>* st_block_segment) {
  const double length = obstacle.length();
  const double width = obstacle.width();
  double s_l = 0.0, s_u = 0.0;

  bool extend_from_previous = false;
  std::vector<StPoint>* current_seg = nullptr;
  for (const auto& future_point : obstacle.prediction()) {
    if (gp_path.HasOverlapWith(future_point, length, width, &s_l, &s_u)) {
      if (!extend_from_previous) {
        st_block_segment->emplace_back();
        current_seg = &st_block_segment->back();
        extend_from_previous = true;
      }
      current_seg->emplace_back(
          StPoint(s_l, s_u, future_point.stamp - stamp_now_));
    } else {
      extend_from_previous = false;
    }
  }
}

bool StGraph::TopKSearch(const int k) {
  CHECK(k % 2 == 1);
  int num_a_per_side = static_cast<int>(k / 2.0);
  std::vector<double> discrete_a;
  discrete_a.reserve(k);
  discrete_a.emplace_back(0.0);
  for (int i = 0; i < num_a_per_side; ++i) {
    discrete_a.emplace_back(a_max_ * (i + 1) / num_a_per_side);
    discrete_a.emplace_back(a_min_ * (i + 1) / num_a_per_side);
  }

  StNodeWeights weight;
  weight.control = 0.1;
  weight.obstacle = 1;
  weight.ref_v = 0.0;
  StNode::SetWeights(weight);
  StNode::SetReferenceSpeed(15.0);

  LOG(INFO) << "prepare a ok";
  std::unique_ptr<StNode> inital_node =
      std::make_unique<StNode>(init_s_[0], init_s_[1]);
  search_tree_.resize(9);
  search_tree_[0].emplace_back(std::move(inital_node));

  LOG(INFO) << "prepare tree ok";

  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 gen(rd());  // seed the generator
  std::uniform_int_distribution<> distr(9, 81);  // define the range

  TIC;
  for (int i = 0; i < 8; ++i) {
    int valiad_candidate = 0;
    for (int j = 0; j < search_tree_[i].size(); ++j) {
      // TODO: add fallback strategy when size < k
      if (valiad_candidate >= k * k) break;

      for (const auto& a : discrete_a) {
        auto next_node = search_tree_[i][j]->Forward(1.0, a);
        next_node->CalObstacleCost(
            sdf_->SignedDistance(Eigen::Vector2d(next_node->t, next_node->s)));
        printf("(%f, %f)\n", next_node->t, next_node->s);
        // LOG(INFO) << "cost: " << next_node->cost << ", d: "
        //           << sdf_->SignedDistance(
        //                  Eigen::Vector2d(next_node->t, next_node->s));
        if (next_node->cost < 1e5) {
          search_tree_[i + 1].emplace_back(std::move(next_node));
          ++valiad_candidate;
        }
      }
    }
    if (search_tree_[i + 1].empty()) {
      LOG(ERROR) << "cannot find valid path";
      return false;
    }
    std::sort(
        search_tree_[i + 1].begin(), search_tree_[i + 1].end(),
        [](const std::unique_ptr<StNode>& n1,
           const std::unique_ptr<StNode>& n2) { return n1->cost < n2->cost; });
  }
  TOC("Top K search");
  return true;
}

bool StGraph::LocalTopSearch(const int k) {
  CHECK(k % 2 == 1);
  int num_a_per_side = static_cast<int>(k / 2.0);
  std::vector<double> discrete_a;
  discrete_a.reserve(k);
  discrete_a.emplace_back(0.0);
  for (int i = 0; i < num_a_per_side; ++i) {
    discrete_a.emplace_back(a_max_ * (i + 1) / num_a_per_side);
    discrete_a.emplace_back(a_min_ * (i + 1) / num_a_per_side);
  }

  StNodeWeights weight;
  weight.control = 0.1;
  weight.obstacle = 1;
  weight.ref_v = 0.1;
  StNode::SetWeights(weight);
  StNode::SetReferenceSpeed(15.0);

  LOG(INFO) << "prepare a ok";
  std::unique_ptr<StNode> inital_node =
      std::make_unique<StNode>(init_s_[0], init_s_[1]);
  search_tree_.resize(9);
  search_tree_[0].emplace_back(std::move(inital_node));

  LOG(INFO) << "prepare tree ok";

  std::random_device rd;   // obtain a random number from hardware
  std::mt19937 gen(rd());  // seed the generator
  std::uniform_int_distribution<> distr(9, 81);  // define the range

  TIC;
  double t_expand = 0.0;
  double t_sort = 0.0;
  double t_compare = 0.0;
  double kEpsilon = 0.1;
  for (int i = 0; i < 8; ++i) {
    std::vector<std::unique_ptr<StNode>> cache;

    auto t0 = std::chrono::high_resolution_clock::now();
    LOG(INFO) << "expand num: " << search_tree_[i].size() * discrete_a.size();
    for (int j = 0; j < search_tree_[i].size(); ++j) {
      for (const auto& a : discrete_a) {
        auto next_node = search_tree_[i][j]->Forward(1.0, a);
        if (next_node->v < 0) continue;  // TODO: can optimize
        for (int k = 1; k <= 5; ++k) {
          double tt, ss, dd;
          next_node->CalObstacleCost(sdf_->SignedDistance(
              Eigen::Vector2d(search_tree_[i][j]->t + k / 5.0,
                              search_tree_[i][j]->GetDistance(k / 5.0, a))));
        }
        if (next_node->cost < 1e5) {
          cache.emplace_back(std::move(next_node));
        }
      }
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    t_expand +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
    std::sort(cache.begin(), cache.end(),
              [](const std::unique_ptr<StNode>& n1,
                 const std::unique_ptr<StNode>& n2) { return n1->s < n2->s; });
    auto t2 = std::chrono::high_resolution_clock::now();
    t_sort +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();

    if (cache.empty()) {
      LOG(ERROR) << "cannot find valid path";
      return false;
    }

    int min_index = 0;
    int min_cost = cache[0]->cost;
    bool is_new_group = false;

    for (size_t j = 0; j < cache.size(); ++j) {
      if (cache[j]->s - cache[min_index]->s <= kEpsilon) {
        if (cache[j]->cost < min_cost) {
          min_cost = cache[j]->cost;
          min_index = j;
        }
      } else {
        is_new_group = true;
      }

      if (is_new_group || j == cache.size() - 1) {
        search_tree_[i + 1].emplace_back(std::move(cache[min_index]));
        min_index = j + 1;
        is_new_group = false;
      }
    }
    auto t3 = std::chrono::high_resolution_clock::now();
    t_compare +=
        std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count();

    kEpsilon *= 1.5;
  }
  std::sort(
      search_tree_.back().begin(), search_tree_.back().end(),
      [](const std::unique_ptr<StNode>& n0, const std::unique_ptr<StNode>& n2) {
        return n0->cost < n2->cost;
      });
  TOC("Local Top K search");
  LOG(INFO) << "t sorting: " << t_sort / 1e6;
  LOG(INFO) << "t expand: " << t_expand / 1e6;
  LOG(INFO) << "t compare: " << t_compare / 1e6;
  return true;
}

void StGraph::VisualizeStGraph() {
  vector_Eigen<Eigen::Vector2d> corners;
  corners.resize(4);
  for (const auto& st_block_segment : st_block_segments_) {
    if (st_block_segment.empty()) continue;
    for (const auto& st_points : st_block_segment) {
      if (st_points.size() <= 1) continue;
      for (size_t i = 0; i < st_points.size() - 1; ++i) {
        std::vector<double> obs_x, obs_y;
        corners.clear();
        corners.emplace_back(st_points[i].t, st_points[i].s_l);
        corners.emplace_back(st_points[i].t, st_points[i].s_u);
        corners.emplace_back(st_points[i + 1].t, st_points[i + 1].s_u);
        corners.emplace_back(st_points[i + 1].t, st_points[i + 1].s_l);
        for (size_t i = 0; i <= corners.size(); ++i) {
          if (i == corners.size()) {
            obs_x.emplace_back(corners[0].x());
            obs_y.emplace_back(corners[0].y());
          } else {
            obs_x.emplace_back(corners[i].x());
            obs_y.emplace_back(corners[i].y());
          }
        }
        plt::plot(obs_x, obs_y, "b");
      }
    }
  }

  std::vector<double> t, s;
  for (int i = 1; i < search_tree_.size(); ++i) {
    if (search_tree_[i].empty()) break;
    for (const auto& node : search_tree_[i]) {
      if (node->parent == nullptr) continue;
      t.clear();
      s.clear();
      for (double i = 0; i < 1; i += 0.1) {
        t.emplace_back(node->parent->t + i);
        s.emplace_back(node->parent->s + node->parent->v * i +
                       0.5 * node->a * i * i);
      }
      plt::plot(t, s, "gray");
    }
  }

  // optimal s-t curve
  const StNode* current_node = search_tree_.back().front().get();
  while (current_node->parent) {
    t.clear();
    s.clear();
    for (double i = 0; i < 1; i += 0.1) {
      t.emplace_back(current_node->parent->t + i);
      s.emplace_back(current_node->parent->s + current_node->parent->v * i +
                     0.5 * current_node->a * i * i);
    }
    plt::plot(t, s, "r");
    current_node = current_node->parent;
  }

  double v0 = init_s_[1];
  double a_max = 2;
  std::vector<double> s_min, s_max;
  t.clear();
  for (double i = 0; i < 8; i += 0.1) {
    t.emplace_back(i);
    s_max.emplace_back(init_s_[0] + v0 * i + 0.5 * a_max * i * i);
    s_min.emplace_back(init_s_[0] + v0 * i - 0.5 * a_max * i * i);
  }

  plt::plot(t, s_min, "r");
  plt::plot(t, s_max, "r");

  plt::xlim(0, 9);
  plt::ylim(0, 105);
  plt::show();
}

}  // namespace planning
