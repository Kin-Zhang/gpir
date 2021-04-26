/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#pragma once

#include <string>

#include "sensor_msgs/PointCloud2.h"

namespace hdmap {
class PointCloudLoader {
 public:
  static bool LoadPcdFile(const std::string& pcd_file,
                          sensor_msgs::PointCloud2* pointcloud);
};
}  // namespace hdmap
