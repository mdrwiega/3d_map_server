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
    bool show_visualizer{false};
    bool output_to_file{true};

    float fitness_score_thresh{0.0001};
    bool icp_correction{true};
    std::string output_dir;
    ICP::Config icp;
    FeaturesMatching::Config template_alignment;

    std::string toString();
    std::vector<std::string> getHeader();
  };

  struct Result {
    float fitness_score{0};
    float fitness_score2{0};
    float fitness_score3{0};
    Point model_min{};
    Point model_max{};
    Eigen::Matrix4f transformation{};

    float transf_estimation_time_ms{0};
    float octree_transformation_time_ms{0};
    float octrees_merge_time_ms{0};

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

  std::string DumpConfigAndResultsToFile();

 private:
  std::vector<Rectangle> spiral_blocks_;
  PointCloudPtr model_;
  PointCloudPtr scene_;
  OcTreePtr model_tree_;
  OcTreePtr scene_tree_;
  Config cfg_;
  Result result_;
};

} // namespace octomap_tools
