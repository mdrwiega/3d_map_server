#include <octomap_tools/feature_cloud.h>

#include <chrono>

#include <pcl/keypoints/iss_3d.h>

#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/filters/uniform_sampling.h>

namespace octomap_tools {

FeatureCloud::FeatureCloud(PointCloud::Ptr cloud, Config config) :
    cloud_(cloud),
    cfg_(config) {
  }

PointCloud::Ptr FeatureCloud::GetPointCloud() const {
  return cloud_;
}

FeatureCloud::SurfaceNormals::Ptr FeatureCloud::GetSurfaceNormals() {
  if (normals_ == nullptr) {
    ComputeSurfaceNormals();
  }
  return normals_;
}

PointCloud::Ptr FeatureCloud::GetKeypoints() {
  if (keypoints_ == nullptr) {
    ExtractKeypoints();
  }
  return keypoints_;
}

FeatureCloud::Descriptors::Ptr FeatureCloud::GetDescriptors() {
  if (descriptors_ == nullptr) {
    ComputeDescriptors();
  }
  return descriptors_;
}

void FeatureCloud::ProcessInput() {
  ComputeSurfaceNormals();
  ExtractKeypoints();
  ComputeDescriptors();
}

void FeatureCloud::ComputeSurfaceNormals() {
  auto start = std::chrono::high_resolution_clock::now();

  normals_ = SurfaceNormals::Ptr(new SurfaceNormals);
  pcl::NormalEstimationOMP<Point, NormalType> norm_est;
  norm_est.setKSearch (cfg_.normal_radius);
  norm_est.setInputCloud (cloud_);
  norm_est.compute (*normals_);

  if (cfg_.debug) {
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    std::cout << "Surface normals computed in " << diff.count() << " ms." << std::endl;
  }
}

void FeatureCloud::ExtractKeypoints() {
  auto start = std::chrono::high_resolution_clock::now();
  keypoints_ = PointCloud::Ptr(new PointCloud());
  if (cfg_.keypoints_method == KeypointsExtractionMethod::Uniform) {
    pcl::UniformSampling<Point> uniform_sampling;
    uniform_sampling.setInputCloud (cloud_);
    uniform_sampling.setRadiusSearch (cfg_.downsampling_radius);
    uniform_sampling.filter(*keypoints_);
  }
  else if (cfg_.keypoints_method == KeypointsExtractionMethod::Iss3d) {
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

  FilterOutNaNs(keypoints_, cfg_.debug);

  if (cfg_.debug) {
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    std::cout << "Extracted " << keypoints_->size () << " keypoints in "
              << diff.count() << " ms." << std::endl;
  }
}

void FeatureCloud::ComputeDescriptors() {
  // Check normals and keypoints
  if (normals_ == nullptr) {
    ComputeSurfaceNormals();
  }
  if (keypoints_ == nullptr) {
    ExtractKeypoints();
  }

  auto start = std::chrono::high_resolution_clock::now();
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

  if (cfg_.debug) {
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    std::cout << "Descriptors computed in: " << diff.count() << " ms." << std::endl;
  }
}

}