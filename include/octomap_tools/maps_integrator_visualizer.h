/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <Eigen/Dense>
#include <pcl/point_types.h>

#include <octomap_tools/utils.h>

namespace octomap_tools {

class MapsIntegratorVisualizer {
 public:
  struct Config {
    bool show_keypoints{false};
    bool save_to_file {false};
    std::string filename;
  };

  MapsIntegratorVisualizer(Config cfg = {false, false, ""}) :
    cfg_(cfg) {
  }

  void visualize(const Eigen::Matrix4f& transformation, PointCloudPtr& scene,
                 PointCloudPtr& model, PointCloudPtr& full_model, std::vector<Rectangle> blocks);

  void visualizeICP(PointCloudPtr& scene, PointCloudPtr& model, const Eigen::Matrix4f& transformation);

  void visualizeClouds(const PointCloudPtr& cloud1, const PointCloudPtr& cloud2);

  Config cfg_;
};

}
