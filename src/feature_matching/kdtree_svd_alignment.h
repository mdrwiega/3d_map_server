#pragma once

#include <Eigen/Dense>
#include <pcl/correspondence.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/transformation_estimation_svd.h>

#include <feature_matching/alignment_method.h>
#include <feature_matching/feature_cloud.h>
#include <validation.h>

namespace octomap_tools {

pcl::CorrespondencesPtr FindFeaturesCorrespondencesWithKdTree(
      const FeatureCloud::Descriptors::Ptr& model_descriptors,
      const FeatureCloud::Descriptors::Ptr& scene_descriptors, float desc_dist_thresh);

class KdTreeBasedAlignment : public AlignmentMethod {
 public:

  struct Config {
    float desc_dist_thresh;
  };

  KdTreeBasedAlignment(const Config& cfg)
    : descriptor_dist_thresh_(cfg.desc_dist_thresh) {
  }

  AlignmentMethod::Result align(const FeatureCloudPtr& model, const FeatureCloudPtr& scene) {
    pcl::CorrespondencesPtr features_correspondences(new pcl::Correspondences);

    // Find corresponding features in the target cloud
    features_correspondences = FindFeaturesCorrespondencesWithKdTree(
      model->GetDescriptors(), scene->GetDescriptors(), descriptor_dist_thresh_);

    // Get sample indices from correspondences
    std::vector<int> sample_indices(features_correspondences->size());
    std::vector<int> corresponding_indices(features_correspondences->size());
    for (const auto& corr : *features_correspondences) {
      sample_indices.push_back(corr.index_query);
      corresponding_indices.push_back(corr.index_match);
    }

    // Estimate transformation with LS SVD method
    pcl::registration::TransformationEstimationSVD<Point, Point> transformation_estimator;
    Eigen::Matrix4f transformation_matrix;
    transformation_estimator.estimateRigidTransformation(
      *model->GetKeypoints(), sample_indices, *scene->GetKeypoints(),
      corresponding_indices, transformation_matrix);

    AlignmentMethod::Result result;
    result.features_correspondences = features_correspondences;
    result.fitness_score = calcFitnessScore1(features_correspondences);
    result.transformation = transformation_matrix;

    std::cout << "\nCorrespondences : " << features_correspondences->size();
    std::cout << "\nFitnessScore1 corr1: " << result.fitness_score << "\n";

    return result;
  }

 private:
  float descriptor_dist_thresh_;
};

  /**
   * Finds correspondences between two sets of descriptors
   */
  pcl::CorrespondencesPtr inline FindFeaturesCorrespondencesWithKdTree(
      const FeatureCloud::Descriptors::Ptr& model_descriptors,
      const FeatureCloud::Descriptors::Ptr& scene_descriptors, float desc_dist_thresh) {

    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

    pcl::KdTreeFLANN<FeatureCloud::DescriptorType> match_search;
    match_search.setInputCloud(model_descriptors);

    //  For each scene keypoint descriptor, find nearest neighbor
    //  into the model keypoints descriptor cloud and add it to the correspondences vector.
    for (size_t i = 0; i < scene_descriptors->size (); ++i) {
      std::vector<int> neigh_indices(1);
      std::vector<float> neigh_sqr_dists(1);
      if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) { //skipping NaNs
        continue;
      }

      int found_neighs = match_search.nearestKSearch(scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);

      // add match only if the squared descriptor distance is
      // less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
      if(found_neighs == 1 && neigh_sqr_dists[0] < desc_dist_thresh) {
        pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
        correspondences->push_back(corr);
      }
    }
    std::cout << "Correspondences found: " << correspondences->size();
    return correspondences;
  }

} // namespace octomap_tools