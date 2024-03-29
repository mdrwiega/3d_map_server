#pragma once

#include <Eigen/Dense>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <alignment/alignment_method.h>
#include <alignment/feature_matching/feature_cloud.h>
#include <alignment/feature_matching/kdtree_svd_alignment.h>
#include <alignment/alignment_validator.h>

namespace octomap_tools {

class GeometryClusteringAlignment : public AlignmentMethod {
 public:

  struct Config {
  };

  GeometryClusteringAlignment(
      const Config& cfg, const FeatureCloudPtr& scene, const FeatureCloudPtr& model)
    : scene_(scene)
    , model_(model) {
  }

  AlignmentMethod::Result Align() {
    float cg_size(0.01f);
    float cg_thresh(5.0f);
    pcl::CorrespondencesPtr features_correspondences(new pcl::Correspondences);

    // Find correspondences with KdTree
    features_correspondences = FindFeaturesCorrespondencesWithKdTree(
      model_->GetDescriptors(), scene_->GetDescriptors(), 1.0);

    // Clustering
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
    std::vector<pcl::Correspondences> clustered_corrs;

    pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
    gc_clusterer.setGCSize(cg_size);
    gc_clusterer.setGCThreshold (cg_thresh);

    gc_clusterer.setInputCloud(model_->GetKeypoints());
    gc_clusterer.setSceneCloud(scene_->GetKeypoints());
    gc_clusterer.setModelSceneCorrespondences(features_correspondences);

    gc_clusterer.cluster(clustered_corrs);
    gc_clusterer.recognize(transformations, clustered_corrs);

    // Find best model - TODO it's common for Hough 3D clustering also
    float lowest_score = std::numeric_limits<float>::infinity();
    size_t best_result_index = 0;

    PCL_DEBUG("\nModel instances found: %d\n", transformations.size());
    for (size_t i = 0; i < transformations.size (); ++i) {
      AlignmentValidator<Point> validator;
      validator.calculateCorrespondences(
        model_->GetPointCloud(), scene_->GetPointCloud(), transformations[i]);
      float fs1 = validator.calcFitnessScore1();
      if (fs1 < lowest_score) {
        lowest_score = fs1;
        best_result_index = i;
      }
      PCL_DEBUG("  Instance %d: correspondences num: %d, fs1: %.2f\n", i + 1, clustered_corrs[i].size(), fs1);
    }

    AlignmentMethod::Result result;
    result.features_correspondences = features_correspondences;
    if (transformations.size() > best_result_index) {
      result.transformation = transformations[best_result_index];
      AlignmentValidator<Point> validator;
      validator.calculateCorrespondences(
        model_->GetPointCloud(), scene_->GetPointCloud(), result.transformation);
      result.fitness_score1 = validator.calcFitnessScore1();
      result.fitness_score2 = validator.calcFitnessScore2();
      result.fitness_score3 = validator.calcFitnessScore3();
    }
    return result;
  }

private:
  FeatureCloudPtr scene_;
  FeatureCloudPtr model_;
};

} // namespace octomap_tools