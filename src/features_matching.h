#pragma once

#include <limits>
#include <chrono>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/correspondence.h>

#include <feature_matching/feature_cloud.h>
#include <octomap_tools/types.h>

#include <feature_matching/alignment_method.h>
#include <feature_matching/kdtree_svd_alignment.h>
#include <feature_matching/sample_consensus_alignment.h>
#include <feature_matching/geometry_clustering_alignment.h>
#include <feature_matching/hough_3d_clustering_alignment.h>

namespace octomap_tools {

class FeaturesMatching {
 public:
  enum class AlignmentMethodType {
    SampleConsensus, Hough3DClustering, GeometryConsistencyClustering, KdTreeSearch };

  struct Config {
    FeatureCloud::Config feature_cloud;
    AlignmentMethodType method = AlignmentMethodType::SampleConsensus;

    bool divide_model{true};

    KdTreeBasedAlignment::Config kdts;        // Kd Tree + SVD
    SampleConsensusAlignment::Config sac;     // Sample Consensus
    GeometryClusteringAlignment::Config gc;   // Geometry Clustering
    Hough3dClusteringAlignment::Config hough; // Hough 3D Clustering

    float cell_size_x;
    float cell_size_y;
    int model_size_thresh_;
    int keypoints_thresh_;

    bool show_visualizer{false};
    bool output_to_file{true};
    std::string output_dir;
  };

  struct Result {
    float fitness_score1{std::numeric_limits<float>::max()};
    float fitness_score2{std::numeric_limits<float>::max()};
    float fitness_score3{std::numeric_limits<float>::max()};
    Eigen::Matrix4f transformation;
    float processing_time_ms;
    pcl::CorrespondencesPtr correspondences;
    pcl::CorrespondencesPtr features_correspondences;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  struct ThreadResult {
    bool valid{false};
    int thread_num{0};
    Result result;
  };

  FeaturesMatching(const Config& config, PointCloudPtr& scene, PointCloudPtr& model);

  Result DivideModelAndAlign(PointCloud& best_model);

  /**
   * Align model feature cloud to scene feature cloud and return estimated transformation.
   * It uses Sample Consensus Initial Alginment method
   */
  static Result Align(
    int nr, FeaturesMatching::Config& cfg, const FeatureCloudPtr& model, const FeatureCloudPtr& scene);

 protected:
  /**
   * Threaded version of alignment method
   */
  static ThreadResult AlignmentThread(
    int nr, PointCloudPtr& full_model, Rectangle block,
    const FeatureCloudPtr& scene, FeaturesMatching::Config& cfg);


  ThreadResult FindBestAlignment(const std::vector<FeaturesMatching::ThreadResult>& results);

 private:
  PointCloudPtr scene_cloud_;
  PointCloudPtr model_cloud_;
  Config cfg_;
};

} // namespace octomap_tools