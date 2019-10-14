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
    bool show_keypoints_ {true};
    bool dump_to_file_{true};
    bool integrate_octomaps {false};
    bool show_integrated_octomaps {false};
    bool show_two_pointclouds{false};

    float fitness_score_thresh{0.0001};
    bool icp_correction{true};
    std::string files_path_and_pattern;
    ICP::Config icp;
    FeaturesMatching::Config template_alignment;

    std::string toString();
    std::string toTable();
    std::vector<std::string> getHeader();
  };

  class Result {
   public:
    float time_ms;
    float fitness_score;
    Point model_min;
    Point model_max;
    Eigen::Matrix4f transformation;

    Result() :
      time_ms(0), fitness_score(0), model_min{}, model_max{}, transformation{} {}

      Result(float _time_ms, float _fitness_score, Point _model_min, Point _model_max, Eigen::Matrix4f _tf);

      std::string toString();

      void PrintResult();
  };

  MapsIntegrator(const OcTreePtr& scene_tree, const OcTreePtr& model_tree, const Config& config) :
    model_tree_(model_tree),
    scene_tree_(scene_tree),
    cfg_(config) {
    model_ = OcTreeToPointCloud(*model_tree_);
    scene_ = OcTreeToPointCloud(*scene_tree_);
  }

  Result compute();

  OcTreePtr integrateOctrees(const Eigen::Matrix4f& transformation);

  void DumpConfigAndResultsToFile();

  std::vector<Rectangle> spiral_blocks_;
  PointCloudPtr model_;
  PointCloudPtr scene_;
  OcTreePtr model_tree_;
  OcTreePtr scene_tree_;
  Config cfg_;
  Result result_;
};

}
