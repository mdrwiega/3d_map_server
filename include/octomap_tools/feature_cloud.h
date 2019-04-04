/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/
#pragma once

#include <chrono>

#include <Eigen/Dense>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/keypoints/narf_keypoint.h>
//#include <pcl/features/fpfh_omp.h>

#include <octomap_tools/utils.h>

namespace octomap_tools {

class FeatureCloud {
 public:
  using NormalType = pcl::Normal;
  using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
  using DescriptorType =  pcl::SHOT352; // pcl::FPFHSignature33;
  using Descriptors = pcl::PointCloud<DescriptorType>;
  using RFType = pcl::ReferenceFrame;

  struct Config {
    float normal_radius = 10.0;
    float downsampling_radius = 0.1;
    float descriptors_radius = 1.5;
  };

  FeatureCloud() = default;

  FeatureCloud(Config config) :
    cfg_(config) {
  }

  FeatureCloud(PointCloud::Ptr cloud, Config config) :
    cloud_(cloud),
    cfg_(config) {
  }

  ~FeatureCloud() = default;

  void setInputCloud (PointCloud::Ptr cloud) {
    cloud_ = cloud;
    processInput();
  }

  // Load and process the cloud in the given PCD file
  void loadInputCloud (const std::string &pcd_file) {
    cloud_ = PointCloud::Ptr (new PointCloud);
    pcl::io::loadPCDFile (pcd_file, *cloud_);
    processInput();
  }

  PointCloud::Ptr getPointCloud() const {
    return (cloud_);
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
    auto start = std::chrono::high_resolution_clock::now();

    normals_ = SurfaceNormals::Ptr(new SurfaceNormals);
    pcl::NormalEstimationOMP<Point, NormalType> norm_est;
    norm_est.setKSearch (cfg_.normal_radius);
    norm_est.setInputCloud (cloud_);
    norm_est.compute (*normals_);

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    std::cout << "Surface normals computed in: " << diff.count() << " ms." << std::endl;
  }

  void downsampleAndExtractKeypoints() {
    auto start = std::chrono::high_resolution_clock::now();

    pcl::UniformSampling<Point> uniform_sampling;
    uniform_sampling.setInputCloud (cloud_);
    uniform_sampling.setRadiusSearch (cfg_.downsampling_radius);
    keypoints_ = PointCloud::Ptr(new PointCloud());
    uniform_sampling.filter(*keypoints_);

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    std::cout << "From cloud (size: " << cloud_->size() << ") extracted "
        << keypoints_->size () << " keypoints in: "
        << diff.count() << " ms." << std::endl;
  }

  void computeDescriptors() {
    auto start = std::chrono::high_resolution_clock::now();

    descriptors_ = Descriptors::Ptr(new Descriptors);
    pcl::SHOTEstimationOMP<Point, NormalType, DescriptorType> descr_est;
    //    pcl::FPFHEstimation<Point, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (cfg_.descriptors_radius);
    descr_est.setInputCloud (keypoints_);
    descr_est.setInputNormals (normals_);
    descr_est.setSearchSurface (cloud_);
    descr_est.compute (*descriptors_);

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    std::cout << "Descriptors computed in: " << diff.count() << " ms." << std::endl;
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
