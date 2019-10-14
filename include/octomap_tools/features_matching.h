#pragma once

#include <limits>

#include <Eigen/Dense>
#include <pcl/point_types.h>

#include <octomap_tools/feature_cloud.h>
#include <octomap_tools/spiral_generator.h>

namespace octomap_tools {

class FeaturesMatching {
 public:
  struct Config{
    float min_sample_distance;
    float max_correspondence_distance;
    float fitness_score_dist{0.5};
    unsigned nr_iterations;
    float cell_size_x{2};
    float cell_size_y{2};
    unsigned model_size_thresh_{50};
    unsigned keypoints_thresh_{10};
    FeatureCloud::Config feature_cloud;
  };

  struct Result {
    float fitness_score {std::numeric_limits<float>::max()};
    Eigen::Matrix4f transformation;
    float processing_time_ms;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  struct ThreadResult {
    bool valid{false};
    int thread_num{0};
    float fitness_score {std::numeric_limits<float>::max()};
    Eigen::Matrix4f transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  FeaturesMatching(const Config& config, PointCloudPtr& scene, PointCloudPtr& model) :
    cfg_(config),
    scene_cloud_(scene),
    model_cloud_(model) {
  }

  ~FeaturesMatching() = default;

  FeaturesMatching::Result initialAlignment(PointCloud& best_model);

  FeaturesMatching::Result Align(int nr, FeaturesMatching::Config& cfg,
                                 const FeatureCloudPtr model, const FeatureCloudPtr scene);

 protected:
  std::vector<Rectangle> divideFullModelIntoBlocks(float block_size_x, float block_size_y);

  FeaturesMatching::ThreadResult findBestAlignment(
      const std::vector<FeaturesMatching::ThreadResult>& results);

 private:
  Config cfg_;
  PointCloudPtr model_cloud_;
  PointCloudPtr scene_cloud_;
};

}
