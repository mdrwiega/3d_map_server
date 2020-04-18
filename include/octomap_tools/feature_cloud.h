#pragma once

#include <pcl/common/common.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/normal_3d_omp.h>

#include <octomap_tools/utils.h>

namespace octomap_tools {

class FeatureCloud {
 public:
  using NormalType = pcl::Normal;
  using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
  using DescriptorType =  pcl::SHOT352; // pcl::FPFHSignature33;
  using Descriptors = pcl::PointCloud<DescriptorType>;
  using RFType = pcl::ReferenceFrame;

  enum class KeypointsExtractionMethod { Uniform, Iss3d };

  struct Config {
    float normal_radius = 10.0;
    float downsampling_radius = 0.1;
    float descriptors_radius = 1.5;
    KeypointsExtractionMethod keypoints_method = KeypointsExtractionMethod::Uniform;
    float iss_model_resolution = 0.05;
    int iss_min_neighbours = 4;
    int iss_num_of_threads = 8;
    float iss_threshold21 = 0.975;
    float iss_threshold32 = 0.975;
    bool debug = false;
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

}
