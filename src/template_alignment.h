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

#include "utils/octree_utils.h"
#include <utils/pointcloud_utils.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/registration/ia_ransac.h>

#include <feature_cloud.h>
#include <utils/spiral_generator.h>

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
    float fitness_score;
    Eigen::Matrix4f final_transformation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  TemplateAlignment(Config config) :
    cfg_(config) {
    // Initialize the parameters in the Sample Consensus Initial Alignment (SAC-IA) algorithm
    sac_ia_.setMinSampleDistance(cfg_.min_sample_distance);
    sac_ia_.setMaxCorrespondenceDistance(cfg_.max_correspondence_distance);
    sac_ia_.setMaximumIterations(cfg_.nr_iterations);
  }

  ~TemplateAlignment() = default;

  // Set the given cloud as the target to which the templates will be aligned
  void setTargetCloud (FeatureCloud &target_cloud) {
    target_ = target_cloud;
    sac_ia_.setInputTarget (target_cloud.getKeypoints());
    sac_ia_.setTargetFeatures (target_cloud.getDescriptors());
  }

  // Add the given cloud to the list of template clouds
  void addTemplateCloud (FeatureCloud &template_cloud) {
    templates_.push_back (template_cloud);
  }

  FeatureCloud getTemplate(unsigned idx) {
    if (idx < templates_.size()) {
      return templates_[idx];
    }
    return FeatureCloud();
  }

  // Align the given template cloud to the target specified by setTargetCloud ()
  void align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
  {
    auto start = std::chrono::high_resolution_clock::now();

    sac_ia_.setInputSource(template_cloud.getKeypoints ());
    sac_ia_.setSourceFeatures (template_cloud.getDescriptors ());

    pcl::PointCloud<pcl::PointXYZ> registration_output;
    sac_ia_.align (registration_output);

    result.fitness_score = (float) sac_ia_.getFitnessScore(cfg_.max_correspondence_distance);
    result.final_transformation = sac_ia_.getFinalTransformation ();

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    std::cout << "Clouds aligned in: " << diff.count() << " ms." << std::endl;
  }

  // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
  void alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
  {
    results.resize (templates_.size ());
    for (size_t i = 0; i < templates_.size (); ++i)
    {
      Point pmin, pmax;
      pcl::getMinMax3D(*templates_[i].getPointCloud(), pmin, pmax);
      std::cout << "Start aligning template nr: " << i << " with " << templates_[i].getKeypoints()->size() << " keypoints\n";
      std::cout << "Model MIN: (" << pmin.x << ", " << pmin.y << ", " << pmin.z << ")  ";
      std::cout << "MAX: (" << pmax.x << ", " << pmax.y << ", " << pmax.z << ")\n";

      align (templates_[i], results[i]);
    }
  }

  // Align all of template clouds to the target cloud to find the one with best alignment score
  int findBestAlignment (TemplateAlignment::Result &result) {
    // Align all of the templates to the target cloud
    std::vector<Result, Eigen::aligned_allocator<Result> > results;
    alignAll (results);

    // Find the template with the best (lowest) fitness score
    float lowest_score = std::numeric_limits<float>::infinity ();
    int best_template = 0;
    for (size_t i = 0; i < results.size (); ++i)
    {
      const Result &r = results[i];
      if (r.fitness_score < lowest_score)
      {
        lowest_score = r.fitness_score;
        best_template = (int) i;
      }
    }

    // Output the best alignment
    result = results[best_template];
    return (best_template);
  }

 private:
  // A list of template clouds and the target to which they will be aligned
  std::vector<FeatureCloud> templates_;
  FeatureCloud target_;

  // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
  pcl::SampleConsensusInitialAlignment<Point, Point, DescriptorType> sac_ia_;
  Config cfg_;
};

}
