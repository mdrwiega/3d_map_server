/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <algorithm>
#include <Eigen/Dense>
#include <octomap/octomap.h>

#include "utils/octree_utils.h"
#include <utils/pointcloud_utils.h>
#include <pcl/kdtree/kdtree_flann.h>
<<<<<<< HEAD
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
namespace octomap_tools {

using Point = pcl::PointXYZ;

struct Rectangle {
  Eigen::Vector2f min;
  Eigen::Vector2f max;
};

struct CorrespondenceGroupingConfig {
  bool show_visualization_{false};
  bool show_keypoints_ {true};
  bool show_correspondences_ {true};
  bool use_hough_ {false};
  float model_ss_ {0.3f};
  float scene_ss_ {0.1f};
  float rf_rad_   {1.5};
  float descr_rad_ {1.5};
  float cg_size_ {0.5f};
  float cg_thresh_ {5.0f};
  unsigned correspondences_thresh_ {9};
  unsigned model_size_thresh_{50};
  float cell_size_x_{2};
  float cell_size_y_{2};
};

struct CGResultEntry {
  Point model_min;
  Point model_max;
  Eigen::Matrix4f transformation;
  unsigned correspondences;
};

class CGResultsSet {
 public:
  void AppendResultEntry(const CGResultEntry& entry) {
    results_.push_back(entry);
  }
  void SortResults() {
    auto comparator = [](const CGResultEntry& a, const CGResultEntry& b) {
      return a.correspondences < b.correspondences;
    };
    std::sort(results_.begin(), results_.end(), comparator);
  }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ed4b629... Next part of refactoring
  Eigen::Matrix4f GetBestTransformation() {
    SortResults();
    if (!results_.empty())
      return results_.back().transformation;
    else
      return Eigen::Matrix4f{};
  }

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> bd57a90... Next part of refactoring about feature detection method
=======
>>>>>>> ed4b629... Next part of refactoring
=======
  void PrintBestTransformation() {
    auto T = GetBestTransformation();
    PrintTransformationMatrix(T);
  }

>>>>>>> 97f1049... Tests update. Working state
  void PrintResults() {
    SortResults();
    std::cout << "Results set size: " << results_.size() << std::endl;
    for (auto& result : results_) {

      std::cout << "Model MIN: (" << result.model_min.x << ", " << result.model_min.y << ", " << result.model_min.z << ")" << std::endl;
      std::cout << "Model MAX: (" << result.model_max.x << ", " << result.model_max.y << ", " << result.model_max.z << ")" << std::endl;
      std::cout << "Correspondences: " << result.correspondences;

      PrintTransformationMatrix(result.transformation);
    }
  }
 private:
  void PrintTransformationMatrix(Eigen::Matrix4f& Mat) {
    Eigen::Matrix3f rotation = Mat.block<3,3>(0, 0);
    Eigen::Vector3f translation = Mat.block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
    printf ("\n");
  }
  std::vector<CGResultEntry> results_;
};

class FeatureCloud {
 public:
  using NormalType = pcl::Normal;
  using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
  using DescriptorType = pcl::SHOT352;
  using Descriptors = pcl::PointCloud<DescriptorType>;

  FeatureCloud () :
    normal_radius_ (10.0),
    downsampling_radius_(0.1),
    descriptors_radius_(1.5)
  {}

  ~FeatureCloud() = default;

  // Process the given cloud
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

  // Compute the surface normals
  void computeSurfaceNormals() {
    normals_ = SurfaceNormals::Ptr(new SurfaceNormals);

    pcl::NormalEstimationOMP<Point, NormalType> norm_est;
    norm_est.setKSearch (normal_radius_);
    norm_est.setInputCloud (cloud_);
    norm_est.compute (*normals_);
  }

  void downsampleAndExtractKeypoints()
  {
    pcl::UniformSampling<Point> uniform_sampling;
    uniform_sampling.setInputCloud (cloud_);
    uniform_sampling.setRadiusSearch (downsampling_radius_);
    pcl::PointCloud<int> keypointIndices2;
    uniform_sampling.compute(keypointIndices2);
    keypoints_ = PointCloud::Ptr(new PointCloud());
    pcl::copyPointCloud(*cloud_, keypointIndices2.points, *keypoints_);
    std::cout << "Pointcloud total points: " << cloud_->size ()
              << "; Selected Keypoints: " << keypoints_->size () << std::endl;
  }

  // Compute the local feature descriptors
  void computeDescriptors() {
    descriptors_ = Descriptors::Ptr(new Descriptors);
    pcl::SHOTEstimationOMP<Point, NormalType, DescriptorType> descr_est;
    descr_est.setRadiusSearch (descriptors_radius_);
    descr_est.setInputCloud (keypoints_);
    descr_est.setInputNormals (normals_);
    descr_est.setSearchSurface (cloud_);
    descr_est.compute (*descriptors_);
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

<<<<<<< HEAD
<<<<<<< HEAD
std::vector<Rectangle> generateBlocksInSpiralOrder(Eigen::Vector2f& min, Eigen::Vector2f& max, Eigen::Vector2f& step_xy);

void calc(FeatureCloudPtr& scene, FeatureCloudPtr& model,
          const CorrespondenceGroupingConfig& config, CGResultsSet& results);

=======
=======
std::vector<Rectangle> generateBlocksInSpiralOrder(Eigen::Vector2f& min, Eigen::Vector2f& max, Eigen::Vector2f& step_xy);

>>>>>>> ed4b629... Next part of refactoring
void calc(FeatureCloudPtr& scene, FeatureCloudPtr& model,
          const CorrespondenceGroupingConfig& config, CGResultsSet& results);

>>>>>>> bd57a90... Next part of refactoring about feature detection method
void visualize(
    const CorrespondenceGroupingConfig& config,
    const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,
    const std::vector<pcl::Correspondences>& clustered_corrs,
    FeatureCloudPtr& scene, FeatureCloudPtr& model);
=======

namespace octomap_tools {
>>>>>>> 5841953... Refactoring

struct OctreeIntegrationConf
{
  unsigned max_iter;          // Max number of iterations (ICP)
  float    max_nn_dist;       // Max correspondence distance (NN)
  float    fitness_eps;       // Euclidean fitness epsilon (ICP)
  Point    intersec_margin;   // Margin in intersecting region extraction (m)
  float    transf_eps;        // Transformation epsilon (ICP)
  float    voxel_size;        // Size of voxel after downsampling
};

/**
 * @brief Integrates two octomaps into one.
 *
 * The output octomap is in the frame of tree2.
 */
OcTreePtr integrateOctomaps(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf,
    const Eigen::Matrix4f& T_init, Eigen::Matrix4f& T_fin,
    float& error);

OcTreePtr integrateOctomapsPcl(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf,
    const Eigen::Matrix4f& T_init, Eigen::Matrix4f& T_fin,
    float& error);

Eigen::Matrix4f estimateTransBetweenOctomaps(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf);

Eigen::Matrix4f estimateTransBetweenOctomapsPcl(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf);

Eigen::Matrix4f estimateTransBetweenPointclouds(
    const PointCloud& cloud1, const PointCloud& cloud2,
    const OctreeIntegrationConf& params);

}
