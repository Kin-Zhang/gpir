/* Copyright 2021 Unity-Drive Inc. All rights reserved */

#include "hdmap/pointcloud/pointcloud_loader.h"

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

namespace hdmap {

bool PointCloudLoader::LoadPcdFile(const std::string& pcd_file,
                                   sensor_msgs::PointCloud2* pointcloud) {
  pcl::PointCloud<pcl::PointXYZ> pcd;
  if (pcl::io::loadPCDFile(pcd_file, pcd) == -1) {
    std::cerr << "Fail to load pcd file: " << pcd_file << std::endl;
    return false;
  }

  for (auto& p : pcd) p.y = -p.y;
  pcl::toROSMsg(pcd, *pointcloud);
  return true;
}
}  // namespace hdmap
