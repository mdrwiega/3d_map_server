#pragma once

#include <pcl/common/common.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <octomap_tools/types.h>

namespace octomap_tools {

class FeatureCloud {
 public:
  using NormalType = pcl::Normal;
  using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
  using DescriptorType = pcl::SHOT352; //pcl::FPFHSignature33;
  using Descriptors = pcl::PointCloud<DescriptorType>;
  using RFType = pcl::ReferenceFrame;

  enum class KeypointsExtractionMethod { Uniform, Iss3d };

  struct Config {
    bool debug = false;

    float normal_radius;
    float downsampling_radius;
    float descriptors_radius;
    KeypointsExtractionMethod keypoints_method = KeypointsExtractionMethod::Uniform;

    // ISS 3D
    float iss_salient_radius;
    float iss_non_max_radius;
    float iss_threshold21;
    float iss_threshold32;
    int iss_min_neighbours;
    int iss_num_of_threads;
  };

  FeatureCloud(PointCloud::Ptr cloud, Config config);

  PointCloud::Ptr GetPointCloud() const;

  SurfaceNormals::Ptr GetSurfaceNormals();

  PointCloud::Ptr GetKeypoints();

  Descriptors::Ptr GetDescriptors();

  /**
   * Execute following operations:
   * - computation of surface normals,
   * - extraction of keypoints,
   * - computation of descriptors.
   */
  void ProcessInput();

  void ComputeSurfaceNormals();

  void ExtractKeypoints();

  void ComputeDescriptors();

 private:

  PointCloud::Ptr cloud_;
  SurfaceNormals::Ptr normals_;
  PointCloud::Ptr keypoints_;
  Descriptors::Ptr descriptors_;
  Config cfg_;
};

using FeatureCloudPtr = std::shared_ptr<FeatureCloud>;

} // namespace octomap_tools