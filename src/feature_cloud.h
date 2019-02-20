/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/
#pragma once

#include <chrono>

#include <Eigen/Dense>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <utils/pointcloud_utils.h>

namespace octomap_tools {

class FeatureCloud {
 public:
  using NormalType = pcl::Normal;
  using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
  using DescriptorType =  pcl::SHOT352; // pcl::FPFHSignature33;
  using Descriptors = pcl::PointCloud<DescriptorType>;
  using RFType = pcl::ReferenceFrame;

  struct Config {
    float normal_radius;
    float downsampling_radius;
    float descriptors_radius;
  };

  FeatureCloud() :
    normal_radius_ (10.0),
    downsampling_radius_(0.1),
    descriptors_radius_(1.5) {
  }

  FeatureCloud(Config config) :
    normal_radius_(config.normal_radius),
    downsampling_radius_(config.downsampling_radius),
    descriptors_radius_(config.descriptors_radius) {
  }

  FeatureCloud(PointCloud::Ptr cloud, Config config) :
    cloud_(cloud),
    normal_radius_(config.normal_radius),
    downsampling_radius_(config.downsampling_radius),
    descriptors_radius_(config.descriptors_radius) {
    processInput();
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
    processInput ();
  }

  // Get a pointer to the cloud 3D points
  PointCloud::Ptr getPointCloud () const {
    return (cloud_);
  }

  // Get a pointer to the cloud of 3D surface normals
  SurfaceNormals::Ptr getSurfaceNormals () const {
    return (normals_);
  }

  // Get a pointer to the cloud of keypoints
  PointCloud::Ptr getKeypoints () const {
    return (keypoints_);
  }

  // Get a pointer to the cloud of feature descriptors
  Descriptors::Ptr getDescriptors () const {
    return (descriptors_);
  }

 protected:
  // Compute the surface normals and local features
  void processInput () {
    computeSurfaceNormals();
    downsampleAndExtractKeypoints();
    computeDescriptors();
  }

  void computeSurfaceNormals() {
    auto start = std::chrono::high_resolution_clock::now();

    normals_ = SurfaceNormals::Ptr(new SurfaceNormals);
    pcl::NormalEstimationOMP<Point, NormalType> norm_est;
    norm_est.setKSearch (normal_radius_);
    norm_est.setInputCloud (cloud_);
    norm_est.compute (*normals_);

    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    std::cout << "Surface normals computed in: " << diff.count() << " ms." << std::endl;
  }

  void downsampleAndExtractKeypoints()
  {
    auto start = std::chrono::high_resolution_clock::now();

    pcl::UniformSampling<Point> uniform_sampling;
    uniform_sampling.setInputCloud (cloud_);
    uniform_sampling.setRadiusSearch (downsampling_radius_);
    pcl::PointCloud<int> keypointIndices2;
    uniform_sampling.compute(keypointIndices2);
    keypoints_ = PointCloud::Ptr(new PointCloud());
    pcl::copyPointCloud(*cloud_, keypointIndices2.points, *keypoints_);

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
    descr_est.setRadiusSearch (descriptors_radius_);
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

  // Parameters
  float normal_radius_;
  float downsampling_radius_;
  float descriptors_radius_;
};

using FeatureCloudPtr = std::shared_ptr<FeatureCloud>;

}
