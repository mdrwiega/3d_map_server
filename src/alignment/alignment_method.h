#pragma once

#include <Eigen/Dense>
#include <pcl/correspondence.h>

#include <alignment/feature_matching/feature_cloud.h>

namespace octomap_tools {

class AlignmentMethod {
 public:
  struct Result {
    float fitness_score1 {std::numeric_limits<float>::max()};
    float fitness_score2 {std::numeric_limits<float>::max()};
    float fitness_score3 {std::numeric_limits<float>::max()};
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    float processing_time_ms = 0;
    pcl::CorrespondencesPtr features_correspondences;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  virtual ~AlignmentMethod() = default;

  virtual Result align(const FeatureCloudPtr& model, const FeatureCloudPtr& scene) = 0;
};

} // namespace octomap_tools