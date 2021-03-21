#pragma once

#include <pcl/correspondence.h>

namespace octomap_tools {

inline double calcFitnessScore1(const pcl::CorrespondencesPtr& correspondences) {
  if (correspondences->size() == 0) {
    return std::numeric_limits<double>::max();
  }

  double fitness_score = 0.0;
  for (const auto& corr : *correspondences) {
    fitness_score += corr.distance;
  }
  return fitness_score / correspondences->size();
}

inline double calcFitnessScore2(const pcl::CorrespondencesPtr& correspondences,
                         double max_dist = 10, unsigned min_pairs_num = 10) {
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

inline double calcFitnessScore3(const pcl::CorrespondencesPtr& correspondences,
                         double dist_threshold = 0.1, unsigned min_pairs_num = 10) {
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

} // namespace octomap_tools