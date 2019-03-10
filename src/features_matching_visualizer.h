/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <algorithm>
#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <pcl/point_types.h>

#include "utils/octree_utils.h"
#include <utils/pointcloud_utils.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>

#include <feature_cloud.h>
#include <utils/spiral_generator.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ia_ransac.h>
#include <utils/spiral_generator.h>

namespace octomap_tools {

class FeatureMatchingVisualizer {
 public:
  struct Config {
    bool show_keypoints{false};
    bool save_to_file {false};
    std::string filename;
  };

  FeatureMatchingVisualizer(Config cfg = {false, false, ""}) :
    cfg_(cfg) {
  }

  void visualize(const Eigen::Matrix4f& transformation,
      FeatureCloudPtr& scene, FeatureCloudPtr& model, PointCloudPtr& full_model, std::vector<Rectangle> blocks);

  void visualizeICP(PointCloudPtr& scene, PointCloudPtr& model, const Eigen::Matrix4f& transformation);

  void visualizeClouds(PointCloudPtr& cloud1, PointCloudPtr& cloud2);

  Config cfg_;
};

}
