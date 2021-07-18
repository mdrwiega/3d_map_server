#pragma once

#include <Eigen/Dense>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/features/board.h>
#include <pcl/point_types.h>

#include <alignment/alignment_method.h>
#include <alignment/feature_matching/feature_cloud.h>
#include <alignment/feature_matching/kdtree_svd_alignment.h>
#include <alignment/alignment_validator.h>
#include <octomap_tools/types.h>

namespace octomap_tools {

class Hough3dClusteringAlignment : public AlignmentMethod {
 public:

  typedef pcl::ReferenceFrame RFType;
  typedef pcl::Normal NormalType;


  struct Config {
  };

  Hough3dClusteringAlignment(
      const Config& cfg, const FeatureCloudPtr& scene, const FeatureCloudPtr& model)
    : scene_(scene)
    , model_(model) {
  }

  AlignmentMethod::Result Align() {

    float rf_rad(0.015f);
    float cg_size(0.01f);
    float cg_thresh(5.0f);

    // Find correspondences with KdTree
    pcl::CorrespondencesPtr features_correspondences(new pcl::Correspondences);
    features_correspondences = FindFeaturesCorrespondencesWithKdTree(
      model_->GetDescriptors(), scene_->GetDescriptors(), 1.0);

    // Clustering
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  Compute (Keypoints) Reference Frames only for Hough
    pcl::PointCloud<RFType>::Ptr model__rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene__rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<Point, NormalType, RFType> rf_est;
    rf_est.setFindHoles(true);
    rf_est.setRadiusSearch(rf_rad);

    rf_est.setInputCloud(model_->GetKeypoints());
    rf_est.setInputNormals(model_->GetSurfaceNormals());
    rf_est.setSearchSurface(model_->GetPointCloud());
    rf_est.compute(*model__rf);

    rf_est.setInputCloud(scene_->GetKeypoints());
    rf_est.setInputNormals(scene_->GetSurfaceNormals());
    rf_est.setSearchSurface(scene_->GetPointCloud());
    rf_est.compute (*scene__rf);

    PCL_INFO("Clustering");

    //  Clustering
    pcl::Hough3DGrouping<Point, Point, RFType, RFType> clusterer;
    clusterer.setHoughBinSize(cg_size);
    clusterer.setHoughThreshold(cg_thresh);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);

    clusterer.setInputCloud (model_->GetKeypoints());
    clusterer.setInputRf(model__rf);
    clusterer.setSceneCloud (scene_->GetKeypoints());
    clusterer.setSceneRf(scene__rf);
    clusterer.setModelSceneCorrespondences(features_correspondences);

    //clusterer.cluster(clustered_corrs);
    clusterer.recognize(transformations, clustered_corrs);

    // Find best model_
    float lowest_score = std::numeric_limits<float>::infinity();
    size_t best_result_index = 0;

    PCL_INFO("model_ instances found: %d", transformations.size());
    for (size_t i = 0; i < transformations.size (); ++i) {
      AlignmentValidator<Point> validator;
      validator.calculateCorrespondences(
        model_->GetPointCloud(), scene_->GetPointCloud(), transformations[i]);
      float fs1 = validator.calcFitnessScore1();
      if (fs1 < lowest_score) {
        lowest_score = fs1;
        best_result_index = i;
      }
      PCL_INFO("  Instance %d: correspondences num: %d, fs1: %.2f\n", i + 1, clustered_corrs[i].size(), fs1);
    }

    AlignmentMethod::Result result;
    result.features_correspondences = features_correspondences;
    result.transformation = transformations[best_result_index];

    AlignmentValidator<Point> validator;
    validator.calculateCorrespondences(
      model_->GetPointCloud(), scene_->GetPointCloud(), result.transformation);
    result.fitness_score1 = validator.calcFitnessScore1();
    result.fitness_score2 = validator.calcFitnessScore2();
    result.fitness_score3 = validator.calcFitnessScore3();

    return result;
  }

 private:
  FeatureCloudPtr scene_;
  FeatureCloudPtr model_;
};

} // namespace octomap_tools