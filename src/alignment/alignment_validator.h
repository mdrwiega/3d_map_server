#pragma once

#include <pcl/correspondence.h>
#include <pcl/search/kdtree.h>

namespace octomap_tools {

template <typename PointType>
class AlignmentValidator {
 public:

  void setCorrespondences(const pcl::CorrespondencesPtr& points_correspondences) {
    distances.resize(points_correspondences->size());
    for (size_t i = 0; i < distances.size(); ++i) {
      distances[i] = std::sqrt((*points_correspondences)[i].distance);
    }
  }

  void calculateCorrespondences(const PointCloudPtr& model, const PointCloudPtr& scene,
      const Eigen::Matrix4f& transformation) {

    // Transform the model
    PointCloud model_transformed;
    transformPointCloud(*model, model_transformed, transformation);

    kd_tree.setInputCloud(scene);

    for (size_t i = 0; i < model_transformed.points.size(); ++i) {
      std::vector<int> nn_indices(1);
      std::vector<float> nn_sqr_dists(1);

      // Find its nearest neighbor in the target
      int k = kd_tree.nearestKSearch(model_transformed.points[i], 1, nn_indices, nn_sqr_dists);
      if(k == 1) {
        distances.push_back(std::sqrt(nn_sqr_dists[0]));
      }
    }
  }

  double calcFitnessScore1() {
    if (distances.size() == 0) {
      return std::numeric_limits<double>::max();
    }

    double fitness_score = 0.0;
    for (const auto& dist : distances) {
      fitness_score += dist;
    }
    return fitness_score / distances.size();
  }

  double calcFitnessScore2(double max_dist = 1.0, unsigned min_pairs_num = 100) {
    double fitness_score = 0.0;
    int cnt = 0;

    for (const auto& dist : distances) {
      if (dist <= max_dist) {
        fitness_score += dist;
        cnt++;
      }
    }

    if (cnt >= min_pairs_num) {
      return fitness_score / cnt;
    }
    return std::numeric_limits<double>::max();
  }

  double calcFitnessScore3(double dist_threshold = 0.3, unsigned min_pairs_num = 100) {
    double fitness_score = 0.0;
    int cnt = 0;
    double mean_dist = 0;
    for (const auto& dist : distances) {
      mean_dist += dist;
    }
    mean_dist /= distances.size();

    for (const auto& dist : distances) {
      if (std::abs(dist - mean_dist) <= dist_threshold) {
        fitness_score += dist;
        cnt++;
      }
    }

    if (cnt >= min_pairs_num) {
      return fitness_score / cnt;
    }
    return std::numeric_limits<double>::max();
  }

 private:
  std::vector<float> distances;
  pcl::search::KdTree<PointType> kd_tree;

};

} // namespace octomap_tools