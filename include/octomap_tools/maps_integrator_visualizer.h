#pragma once

#include <Eigen/Dense>
#include <pcl/point_types.h>

#include <common/utils.h>
#include <feature_matching/feature_cloud.h>

namespace octomap_tools {

class MapsIntegratorVisualizer {
 public:
  struct Config {
    bool screen_mode;
    bool save_to_file;
    std::string filename;
  };

  MapsIntegratorVisualizer(Config cfg = {true, false, ""}) :
    cfg_(std::move(cfg)) {
  }

  /**
   * Visualize ICP matching: scene, model and transformed model
   */
  void VisualizeICP(PointCloudPtr& scene, PointCloudPtr& model, const Eigen::Matrix4f& transformation);

  /**
   * Visualize two point clouds
   */
  void VisualizeClouds(const PointCloudPtr& cloud1, const PointCloudPtr& cloud2);

  /**
   * Visualize feature matching process
   */
  void VisualizeFeatureMatching(
    const FeatureCloudPtr& scene, const FeatureCloudPtr& model,
    const Eigen::Matrix4f& transformation,
    const pcl::CorrespondencesPtr& correspondences);

  /**
   * Visualize feature matching process with divided model
   */
  void VisualizeFeatureMatchingWithDividedModel(
    PointCloudPtr& scene,
    PointCloudPtr& model, PointCloudPtr& full_model,
    const Eigen::Matrix4f& transformation, const std::vector<Rectangle>& blocks);

  /**
   * General visualization
   * It visualize elements only if they are not nullptr.
   * If non-zero transformation is passed then it show also transformed cloud1.
   */
  void Visualize(
    const PointCloudPtr& cloud1 = nullptr,
    const PointCloudPtr& cloud2 = nullptr,
    const PointCloudPtr& cloud3 = nullptr,
    const PointCloudPtr& cloud4 = nullptr,
    const PointCloudPtr& keypoints1 = nullptr,
    const PointCloudPtr& keypoints2 = nullptr,
    const pcl::CorrespondencesPtr& correspondences = nullptr,
    const std::vector<Rectangle>& blocks = {});

  Config cfg_;
};

} // namespace octomap_tools
