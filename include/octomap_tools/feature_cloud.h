/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/
#pragma once

#include <chrono>
#include <exception>

#include <Eigen/Dense>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/filters/extract_indices.h>
#include <octomap_tools/utils.h>

namespace octomap_tools {

class FeatureCloud {
 public:
  using NormalType = pcl::Normal;
  using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
  using DescriptorType =  pcl::SHOT352; // pcl::FPFHSignature33;
  using Descriptors = pcl::PointCloud<DescriptorType>;
  using RFType = pcl::ReferenceFrame;

  enum class KeypointsDetectMethod { Uniform, Iss3d };

  struct Config {
    float normal_radius = 10.0;
    float downsampling_radius = 0.1;
    float descriptors_radius = 1.5;
    KeypointsDetectMethod keypoints_method = KeypointsDetectMethod::Uniform;
    float iss_model_resolution = 0.05;
    int iss_min_neighbours = 4;
    int iss_num_of_threads = 8;
    float iss_threshold21 = 0.975;
    float iss_threshold32 = 0.975;
  };

  FeatureCloud(PointCloud::Ptr cloud, Config config) :
    cloud_(cloud),
    cfg_(config) {
  }

  ~FeatureCloud() = default;

  PointCloud::Ptr getPointCloud() const {
    return cloud_;
  }

  SurfaceNormals::Ptr getSurfaceNormals() const {
    return (normals_);
  }

  PointCloud::Ptr getKeypoints() const {
    return (keypoints_);
  }

  Descriptors::Ptr getDescriptors() const {
    return (descriptors_);
  }

  void processInput() {
    computeSurfaceNormals();
    downsampleAndExtractKeypoints();
    computeDescriptors();
  }

  void computeSurfaceNormals() {
    //    auto start = std::chrono::high_resolution_clock::now();
    normals_ = SurfaceNormals::Ptr(new SurfaceNormals);
    pcl::NormalEstimationOMP<Point, NormalType> norm_est;
    norm_est.setKSearch (cfg_.normal_radius);
    norm_est.setInputCloud (cloud_);
    norm_est.compute (*normals_);

    //    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
    //        std::chrono::high_resolution_clock::now() - start);
    //    std::cout << "Surface normals computed in: " << diff.count() << " ms." << std::endl;
  }

  void downsampleAndExtractKeypoints() {
    //        auto start = std::chrono::high_resolution_clock::now();
    keypoints_ = PointCloud::Ptr(new PointCloud());
    if (cfg_.keypoints_method == KeypointsDetectMethod::Uniform) {
      pcl::UniformSampling<Point> uniform_sampling;
      uniform_sampling.setInputCloud (cloud_);
      uniform_sampling.setRadiusSearch (cfg_.downsampling_radius);
      uniform_sampling.filter(*keypoints_);
    }
    else if (cfg_.keypoints_method == KeypointsDetectMethod::Iss3d) {
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
      pcl::ISSKeypoint3D<pcl::PointXYZ, pcl::PointXYZ> iss_detector;
      iss_detector.setSearchMethod (tree);
      iss_detector.setSalientRadius (6 * cfg_.iss_model_resolution);
      iss_detector.setNonMaxRadius (4 * cfg_.iss_model_resolution);
      iss_detector.setThreshold21 (cfg_.iss_threshold21);
      iss_detector.setThreshold32 (cfg_.iss_threshold32);
      iss_detector.setMinNeighbors (cfg_.iss_min_neighbours);
      iss_detector.setNumberOfThreads (cfg_.iss_num_of_threads);
      iss_detector.setInputCloud (cloud_);
      iss_detector.compute (*keypoints_);
    }

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (size_t i = 0; i < keypoints_->size(); ++i) {
      if (!std::isfinite (keypoints_->points[i].x) || 
          !std::isfinite (keypoints_->points[i].y) || 
          !std::isfinite (keypoints_->points[i].z)) {
        inliers->indices.push_back(i);
      }
    }
    std::cout << "\nRemoved " << inliers->indices.size() << " keypoints with NaN\n";
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(keypoints_);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*keypoints_);

    //    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
    //        std::chrono::high_resolution_clock::now() - start);
    //    std::cout << "From cloud (size: " << cloud_->size() << ") extracted "
    //        << keypoints_->size () << " keypoints in: "
    //        << diff.count() << " ms." << std::endl;
  }

  void computeDescriptors() {
    //    auto start = std::chrono::high_resolution_clock::now();
    descriptors_ = Descriptors::Ptr(new Descriptors);
    pcl::SHOTEstimationOMP<Point, NormalType, DescriptorType> descr_est;
    //    pcl::FPFHEstimation<Point, NormalType, DescriptorType> descr_est;
    
    descr_est.setRadiusSearch (cfg_.descriptors_radius);
    descr_est.setInputCloud (keypoints_);
    descr_est.setInputNormals (normals_);
    descr_est.setSearchSurface (cloud_);
    descr_est.compute (*descriptors_);

    // Remove NaN descriptors and related keypoints
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    for (size_t i = 0; i < descriptors_->size(); ++i) {
      if (pcl_isnan( (*descriptors_)[i].descriptor[0]) || pcl_isnan((*descriptors_)[i].rf[0])) {
        inliers->indices.push_back(i);
      }
    }

  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(keypoints_);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*keypoints_);

  pcl::ExtractIndices<DescriptorType> extract_descriptors;
  extract_descriptors.setInputCloud(descriptors_);
  extract_descriptors.setIndices(inliers);
  extract_descriptors.setNegative(true);
  extract_descriptors.filter(*descriptors_);

    //    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
    //        std::chrono::high_resolution_clock::now() - start);
    //    std::cout << "Descriptors computed in: " << diff.count() << " ms." << std::endl;
  }

 private:
  PointCloud::Ptr cloud_;
  SurfaceNormals::Ptr normals_;
  PointCloud::Ptr keypoints_;
  Descriptors::Ptr descriptors_;
  Config cfg_;
};

using FeatureCloudPtr = std::shared_ptr<FeatureCloud>;

}
