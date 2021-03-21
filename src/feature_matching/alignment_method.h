#pragma once

#include <Eigen/Dense>
#include <pcl/correspondence.h>

#include <feature_matching/feature_cloud.h>

namespace octomap_tools {

class AlignmentMethod {
 public:
  struct Result {
    float fitness_score {std::numeric_limits<float>::max()};
    Eigen::Matrix4f transformation;
    float processing_time_ms;
    pcl::CorrespondencesPtr features_correspondences;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  virtual ~AlignmentMethod() = default;

  virtual Result align(const FeatureCloudPtr& model, const FeatureCloudPtr& scene) = 0;
};

} // namespace octomap_tools