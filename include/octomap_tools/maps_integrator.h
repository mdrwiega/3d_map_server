/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <iostream>
#include <algorithm>

#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <octomap_tools/utils.h>
#include <octomap_tools/conversions.h>
#include <octomap_tools/feature_cloud.h>
#include <octomap_tools/features_matching.h>
#include <octomap_tools/icp.h>
#include <octomap_tools/maps_integrator_visualizer.h>

namespace octomap_tools {

class MapsIntegrator {
  using NormalType = FeatureCloud::NormalType;
  using SurfaceNormals = FeatureCloud::SurfaceNormals;
  using DescriptorType = FeatureCloud::DescriptorType;
  using Descriptors = FeatureCloud::Descriptors;
  using RFType = FeatureCloud::RFType;

 public:
  struct Config {
    bool show_visualization_{false};
    bool show_two_pointclouds{false};
    bool dump_to_file_{true};
    bool debug{true};
 
    float fitness_score_thresh{0.0001};
    bool icp_correction{true};
    std::string files_path_and_pattern;
    ICP::Config icp;
    FeaturesMatching::Config template_alignment;

    std::string toString();
    std::string toTable();
    std::vector<std::string> getHeader();
  };

  struct Result {
    float time_ms{0};
    float fitness_score{0};
    Point model_min{};
    Point model_max{};
    Eigen::Matrix4f transformation{};

    std::string toString();
    void PrintResult();
  };

  MapsIntegrator(const OcTreePtr& scene_tree, const OcTreePtr& model_tree, const Config& config);

  /**
   * Estimate transformation between two maps
   */
  Result EstimateTransformation();

  /**
   * Merge maps based on provided transformation
   */
  OcTreePtr Merge(const Eigen::Matrix4f& transformation, bool save_to_file = false);

  /**
   * Estimate transformation between maps and merge them
   */
  OcTreePtr Merge(bool save_to_file = false);

  void DumpConfigAndResultsToFile();

 private:
  std::vector<Rectangle> spiral_blocks_;
  PointCloudPtr model_;
  PointCloudPtr scene_;
  OcTreePtr model_tree_;
  OcTreePtr scene_tree_;
  Config cfg_;
  Result result_;
};

}
