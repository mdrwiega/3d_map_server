#pragma once

#include <limits>

#include <Eigen/Dense>
#include <pcl/point_types.h>

#include <octomap_tools/feature_cloud.h>
#include <octomap_tools/spiral_generator.h>

namespace octomap_tools {

class FeaturesMatching {
 public:
  enum class AlignmentMethod { SampleConsensus, Hough3DClustering, GeometryConsistencyClustering };

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
    AlignmentMethod method = AlignmentMethod::SampleConsensus;
    bool debug{true};
    bool visualize{false};
  };

  struct Result {
    float fitness_score {std::numeric_limits<float>::max()};
    Eigen::Matrix4f transformation;
    float processing_time_ms;
    pcl::CorrespondencesPtr correspondences;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  struct ThreadResult {
    bool valid{false};
    int thread_num{0};
    float fitness_score {std::numeric_limits<float>::max()};
    Eigen::Matrix4f transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  FeaturesMatching(const Config& config, PointCloudPtr& scene, PointCloudPtr& model);

  FeaturesMatching::Result initialAlignment(PointCloud& best_model);

  /**
   * Align model feature cloud to scene feature cloud and return estimated transformation.
   * It uses Sample Consensus Initial Alginment method
   */
  static FeaturesMatching::Result Align(
    int nr, FeaturesMatching::Config& cfg, const FeatureCloudPtr model, const FeatureCloudPtr scene);

 protected:
 /**
  * Model decomposition into rectangular blocks.
  * Returns vector of rectangles which defines submodels coordinations.
  */
  std::vector<Rectangle> RectangularModelDecomposition(float block_size_x, float block_size_y);

  FeaturesMatching::ThreadResult findBestAlignment(
      const std::vector<FeaturesMatching::ThreadResult>& results);

  static pcl::CorrespondencesPtr FindCorrespondencesWithKdTree(
    FeatureCloud::Descriptors::Ptr model_descriptors, FeatureCloud::Descriptors::Ptr scene_descriptors);

  static void Visualize(
    const FeatureCloudPtr model, const FeatureCloudPtr scene, const FeaturesMatching::Result& result,
    bool show_keypoints = true, bool show_correspondences = true);

 private:
  PointCloudPtr scene_cloud_;
  PointCloudPtr model_cloud_;
  Config cfg_;
};

}
