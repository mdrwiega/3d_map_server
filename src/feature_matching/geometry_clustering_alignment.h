#pragma once

#include <Eigen/Dense>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <feature_matching/alignment_method.h>
#include <feature_matching/feature_cloud.h>
#include <feature_matching/kdtree_svd_alignment.h>
#include <validation.h>

namespace octomap_tools {

class GeometryClusteringAlignment : public AlignmentMethod {
 public:

  struct Config {
  };

  GeometryClusteringAlignment(const Config& cfg) {
  }

  AlignmentMethod::Result align(const FeatureCloudPtr& model, const FeatureCloudPtr& scene) {
    float cg_size(0.01f);
    float cg_thresh(5.0f);
    pcl::CorrespondencesPtr features_correspondences(new pcl::Correspondences);

    // Find correspondences with KdTree
    features_correspondences = FindFeaturesCorrespondencesWithKdTree(
      model->GetDescriptors(), scene->GetDescriptors(), 1.0);

    // Clustering
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
    gc_clusterer.setGCSize(cg_size);
    gc_clusterer.setGCThreshold (cg_thresh);

    gc_clusterer.setInputCloud(model->GetKeypoints());
    gc_clusterer.setSceneCloud(scene->GetKeypoints());
    gc_clusterer.setModelSceneCorrespondences(features_correspondences);

    gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize(rototranslations, clustered_corrs);

    AlignmentMethod::Result result;
    result.fitness_score1 = 0;
    if (rototranslations.size() >= 1) {
      result.transformation = rototranslations[0];
    }
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (size_t i = 0; i < rototranslations.size (); ++i) {
      std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
      std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;
    }

    result.features_correspondences = features_correspondences;

    AlignmentValidator<Point> validator;
    validator.calculateCorrespondences(
      model->GetPointCloud(), scene->GetPointCloud(), result.transformation);
    result.fitness_score1 = validator.calcFitnessScore1();
    result.fitness_score2 = validator.calcFitnessScore2();
    result.fitness_score3 = validator.calcFitnessScore3();

    return result;
  }

 private:
};

} // namespace octomap_tools