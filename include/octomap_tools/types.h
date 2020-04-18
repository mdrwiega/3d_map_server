#pragma once

#include <memory>

#include <pcl/common/common.h>
#include <pcl/point_types.h>

#include <octomap/octomap.h>
#include <Eigen/Dense>

namespace octomap_tools {

using Point = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = PointCloud::Ptr;

using OcTree = octomap::OcTree;
using OcTreePtr = std::shared_ptr<octomap::OcTree>;
using OcTreeNode = octomap::OcTreeNode;
using OcTreeKey = octomap::OcTreeKey;

struct Rectangle {
  Eigen::Vector2f min;
  Eigen::Vector2f max;
};

}
