#pragma once

#include <pcl/correspondence.h>
#include <pcl/search/kdtree.h>

namespace octomap_tools {

template <typename PointType>
class AlignmentValidator {
 public:

  void setCorrespondences(const pcl::CorrespondencesPtr& points_correspondences) {
    correspondences = points_correspondences;
  }

  void calculateCorrespondences(const PointCloudPtr& model, const PointCloudPtr& scene,
      const Eigen::Matrix4f& transformation) {
    correspondences.reset(new pcl::Correspondences);

    // Transform the model
    PointCloud model_transformed;
    transformPointCloud(*model, model_transformed, transformation);

    kd_tree.setInputCloud(scene);

    for (size_t i = 0; i < model_transformed.points.size(); ++i) {
      std::vector<int> nn_indices(1);
      std::vector<float> nn_dists(1);

      // Find its nearest neighbor in the target
      int k = kd_tree.nearestKSearch(model_transformed.points[i], 1, nn_indices, nn_dists);
      if(k == 1) {
        pcl::Correspondence corr(nn_indices[0], static_cast<int>(i), nn_dists[0]);
        correspondences->push_back(corr);
      }
    }
  }

  pcl::CorrespondencesPtr getCorrespondences() { return correspondences; }

  double calcFitnessScore1() {
    if (!correspondences) {
      throw std::runtime_error("calcFitnessScore1: No correspondences");
    }

    if (correspondences->size() == 0) {
      return std::numeric_limits<double>::max();
    }

    double fitness_score = 0.0;
    for (const auto& corr : *correspondences) {
      fitness_score += corr.distance;
    }
    return fitness_score / correspondences->size();
  }

  double calcFitnessScore2(double max_dist = 10, unsigned min_pairs_num = 10) {
    if (!correspondences) {
      throw std::runtime_error("calcFitnessScore2: No correspondences");
    }

    double fitness_score = 0.0;
    int cnt = 0;

    for (const auto& corr : *correspondences) {
      if (corr.distance <= max_dist) {
        fitness_score += corr.distance;
        cnt++;
      }
    }

    if (cnt >= min_pairs_num) {
      return fitness_score / cnt;
    }
    return std::numeric_limits<double>::max();
  }

  double calcFitnessScore3(double dist_threshold = 0.1, unsigned min_pairs_num = 10) {
    if (!correspondences) {
      throw std::runtime_error("calcFitnessScore3: No correspondences");
    }

    double fitness_score = 0.0;
    int cnt = 0;
    double mean_dist = 0;
    for (const auto& corr : *correspondences) {
      mean_dist += corr.distance;
    }
    mean_dist /= correspondences->size();

    for (const auto& corr : *correspondences) {
      if (std::abs(corr.distance - mean_dist) <= dist_threshold) {
        fitness_score += corr.distance;
        cnt++;
      }
    }

    if (cnt >= min_pairs_num) {
      return fitness_score / cnt;
    }
    return std::numeric_limits<double>::max();
  }

 private:
  pcl::CorrespondencesPtr correspondences;
  pcl::search::KdTree<PointType> kd_tree;

};

} // namespace octomap_tools