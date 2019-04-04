/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <chrono>
#include <algorithm>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ia_ransac.h>

#include <octomap_tools/feature_cloud.h>

namespace octomap_tools {

class TemplateAlignment {
  using NormalType = FeatureCloud::NormalType;
  using SurfaceNormals = FeatureCloud::SurfaceNormals;
  using DescriptorType = FeatureCloud::DescriptorType;
  using Descriptors = FeatureCloud::Descriptors;
  using RFType = FeatureCloud::RFType;

 public:
  struct Config{
    float min_sample_distance;
    float max_correspondence_distance;
    unsigned nr_iterations;
  };

  struct Result {
    float fitness_score {std::numeric_limits<float>::max()};
    Eigen::Matrix4f final_transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  TemplateAlignment(Config config) :
    cfg_(config) {
    sac_ia_.setMinSampleDistance(cfg_.min_sample_distance);
    sac_ia_.setMaxCorrespondenceDistance(cfg_.max_correspondence_distance);
    sac_ia_.setMaximumIterations(cfg_.nr_iterations);
  }

  ~TemplateAlignment() = default;

  void setTargetCloud (FeatureCloud &target_cloud) {
    target_ = target_cloud;
    sac_ia_.setInputTarget (target_cloud.getKeypoints());
    sac_ia_.setTargetFeatures (target_cloud.getDescriptors());
  }

  void addTemplateCloud(FeatureCloud &template_cloud) {
    templates_.push_back (template_cloud);
  }

  TemplateAlignment::Result addTemplateCloudAndAlign(FeatureCloud &template_cloud) {
    templates_.push_back (template_cloud);
    return align(template_cloud);
  }

  FeatureCloud getTemplate(unsigned idx) {
    if (idx < templates_.size()) {
      return templates_[idx];
    }
    return FeatureCloud();
  }

  FeatureCloud getBestTemplate() {
    float lowest_score = std::numeric_limits<float>::infinity();
    int best_template = 0;
    for (size_t i = 0; i < results_.size (); ++i) {
      const Result &r = results_[i];
      if (r.fitness_score < lowest_score) {
        lowest_score = r.fitness_score;
        best_template = static_cast<int>(i);
      }
    }
    return templates_[best_template];
  }

  TemplateAlignment::Result align(FeatureCloud &template_cloud)
  {
    auto start = std::chrono::high_resolution_clock::now();

    Point pmin, pmax;
    pcl::getMinMax3D(*template_cloud.getPointCloud(), pmin, pmax);
    std::cout << "Start aligning template with " << template_cloud.getKeypoints()->size() << " keypoints\n";
    std::cout << "Model MIN: (" << pmin.x << ", " << pmin.y << ", " << pmin.z << ")  ";
    std::cout << "MAX: (" << pmax.x << ", " << pmax.y << ", " << pmax.z << ")\n";

    sac_ia_.setInputSource(template_cloud.getKeypoints ());
    sac_ia_.setSourceFeatures (template_cloud.getDescriptors ());

    pcl::PointCloud<pcl::PointXYZ> registration_output;
    sac_ia_.align (registration_output);

    TemplateAlignment::Result result;
    result.fitness_score = (float) sac_ia_.getFitnessScore(cfg_.max_correspondence_distance);
    result.final_transformation = sac_ia_.getFinalTransformation ();

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    std::cout << "Clouds aligned with score: " << result.fitness_score << " in: " << diff.count() << " ms." << std::endl;

    results_.push_back(result);
    return result;
  }

  std::vector<Result, Eigen::aligned_allocator<Result>> alignAll()
  {
    for (size_t i = 0; i < templates_.size (); ++i) {
      auto result = align(templates_[i]);
      results_.push_back(result);
    }
    return results_;
  }

  std::vector<Result, Eigen::aligned_allocator<Result>>& getResults() {
    return results_;
  }

  TemplateAlignment::Result findBestAlignment() {
    float lowest_score = std::numeric_limits<float>::infinity();
    int best_template = 0;
    for (size_t i = 0; i < results_.size (); ++i) {
      const Result &r = results_[i];
      if (r.fitness_score < lowest_score) {
        lowest_score = r.fitness_score;
        best_template = static_cast<int>(i);
      }
    }
    return results_[best_template];
  }



 private:
  std::vector<FeatureCloud> templates_;
  FeatureCloud target_;

  pcl::SampleConsensusInitialAlignment<Point, Point, DescriptorType> sac_ia_;
  Config cfg_;
  std::vector<Result, Eigen::aligned_allocator<Result>> results_;
};

}
