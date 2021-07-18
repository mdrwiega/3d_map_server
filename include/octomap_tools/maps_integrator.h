#pragma once

#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

#include <octomap_tools/maps_integrator_visualizer.h>
#include <alignment/feature_matching/feature_cloud.h>
#include <alignment/features_matching.h>
#include <alignment/icp.h>
#include <alignment/ndt_alignment.h>

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
    std::string output_dir;
    ICP::Config icp;

    bool enable_global_alignment{true};
    bool enable_local_alignment{true};
    GlobalAlignment::Method global_alignment_method;
    LocalAlignment::Method local_alignment_method;
    NdtAlignment::Config ndt_alignment;
    FeaturesMatching::Config template_alignment;

    std::string toString();
    std::vector<std::string> getHeader();
  };

  struct Result {
    Point model_min{};
    Point model_max{};

    float transf_estimation_time_ms{0};
    float octree_transformation_time_ms{0};
    float octrees_merge_time_ms{0};

    AlignmentMethod::Result local;
    AlignmentMethod::Result global;
    AlignmentMethod::Result final;

    std::string toString();
    void PrintResult();
  };

  MapsIntegrator(const OcTreePtr& scene_tree, const OcTreePtr& model_tree, const Config& config);

  /**
   * Estimate transformation between two maps
   *
   * Three modes are possible:
   * - global alignment + local alignment
   * - only global_alignment
   * - only local alignment
   */
  Result EstimateTransformation();


  AlignmentMethod::Result GlobalAlignment(PointCloud::Ptr& best_model);
  AlignmentMethod::Result LocalAlignment(PointCloud::Ptr& scene, PointCloud::Ptr& model);

  /**
   * Merge maps based on provided transformation
   */
  OcTreePtr Merge(const Eigen::Matrix4f& transformation, bool save_to_file = false);

  /**
   * Estimate transformation between maps and merge them
   */
  OcTreePtr Merge(bool save_to_file = false);

  /**
   * Saves parameters/results to file. If filename not specified then path from configuration is used.
   */
  std::string DumpConfigAndResultsToFile(const std::string& filename = {});

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
