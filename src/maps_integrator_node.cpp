#include <vector>
#include <exception>

#include <ros/param.h>
#include <ros/package.h>
#include <ros/console.h>


#include <octomap_tools/octomap_io.h>
#include <octomap_tools/maps_integrator_node.h>

namespace octomap_tools {

MapsIntegrator::Config MapsIntegratorNode::GetConfigFromRosParams() {
  MapsIntegrator::Config cfg;

  // Common
  std::string output_dir;
  nh_.param<std::string>("output_dir", output_dir, "");
  bool output_to_file;
  nh_.param<bool>("output_to_file", output_to_file, false);
  bool show_visualizer;
  nh_.param<bool>("show_visualizer", show_visualizer, false);

  // Main parameters
  cfg.show_visualizer = show_visualizer;

  nh_.param<bool>("global_alignment/divide_model",cfg.template_alignment.divide_model, true);
  nh_.param<float>("global_alignment/block_size", cfg.template_alignment.cell_size_x, 3.0);
  nh_.param<float>("global_alignment/block_size", cfg.template_alignment.cell_size_y, 3.0);

  // Feature matching
  std::string fm_ns = "global_alignment/feature_matching/";
  auto& fm = cfg.template_alignment;
  nh_.param<int>(fm_ns + "min_model_size", fm.model_size_thresh_, 400);
  nh_.param<int>(fm_ns + "min_keypoints_num", fm.keypoints_thresh_, 40);
  nh_.param<float>(fm_ns + "normal_radius", fm.feature_cloud.normal_radius, 10.0);
  nh_.param<float>(fm_ns + "downsampling_radius", fm.feature_cloud.downsampling_radius, 0.15);
  nh_.param<float>(fm_ns + "descriptors_radius", fm.feature_cloud.descriptors_radius, 1.0);
  std::string alignment_method;
  nh_.param<std::string>(fm_ns + "alignment_method", alignment_method, "sac");
  if (alignment_method.compare("sac") == 0)
    fm.method = FeaturesMatching::AlignmentMethodType::SampleConsensus;
  else if (alignment_method.compare("kdts") == 0)
    fm.method = FeaturesMatching::AlignmentMethodType::KdTreeSearch;
  else if (alignment_method.compare("gcc") == 0)
    fm.method = FeaturesMatching::AlignmentMethodType::GeometryConsistencyClustering;
  else if (alignment_method.compare("hough") == 0)
    fm.method = FeaturesMatching::AlignmentMethodType::Hough3DClustering;

  cfg.template_alignment.output_dir = output_dir;
  cfg.template_alignment.show_visualizer = show_visualizer;
  cfg.template_alignment.output_to_file = output_to_file;

  // Sample Consensus
  std::string sac_ns = "global_alignment/feature_matching/sample_consensus/";
  nh_.param<int>(sac_ns + "iterations_num", cfg.template_alignment.sac.nr_iterations, 1000);
  nh_.param<float>(sac_ns + "min_sample_distance", cfg.template_alignment.sac.min_sample_distance, 0.2);
  nh_.param<float>(sac_ns + "max_correspondence_distance", cfg.template_alignment.sac.max_correspondence_distance, 100.0);
  nh_.param<float>(sac_ns + "fitness_score_distance", cfg.template_alignment.sac.fitness_score_dist, 0.5);

  // Keypoints extraction method
  std::string keypoints_method;
  nh_.param<std::string>("global_alignment/feature_matching/keypoints_method", keypoints_method, "iss3d");
  if (keypoints_method.compare("iss3d") == 0)
    cfg.template_alignment.feature_cloud.keypoints_method = FeatureCloud::KeypointsExtractionMethod::Iss3d;
  else if (keypoints_method.compare("uniform") == 0)
    cfg.template_alignment.feature_cloud.keypoints_method = FeatureCloud::KeypointsExtractionMethod::Uniform;

  // ISS 3D
  std::string iss3d_ns = "global_alignment/feature_matching/iss3d/";
  auto& iss3d = cfg.template_alignment.feature_cloud;
  nh_.param<float>(iss3d_ns + "salient_radius", iss3d.iss_salient_radius, 0.12);
  nh_.param<float>(iss3d_ns + "non_max_radius", iss3d.iss_non_max_radius, 0.08);
  nh_.param<float>(iss3d_ns + "threshold21", iss3d.iss_threshold21, 0.975);
  nh_.param<float>(iss3d_ns + "threshold32", iss3d.iss_threshold32, 0.975);
  nh_.param<int>(iss3d_ns + "min_neighbours", iss3d.iss_min_neighbours, 6);
  nh_.param<int>(iss3d_ns + "num_of_threads", iss3d.iss_num_of_threads, 2);

  nh_.param<bool>("local_alignment_enable", cfg.icp_correction, false);
  nh_.param<float>("local_alignment/scene_inflation_dist", cfg.icp.scene_inflation_dist, 2.5);

  // ICP
  std::string icp_ns = "local_alignment/icp/";
  nh_.param<int>(icp_ns + "max_iterations", cfg.icp.max_iter, 500);
  nh_.param<float>(icp_ns + "max_nn_dist", cfg.icp.max_nn_dist, 0.5);
  nh_.param<float>(icp_ns + "fitness_eps", cfg.icp.fitness_eps, 0.0005);
  nh_.param<float>(icp_ns + "fitness_score_dist", cfg.icp.fitness_score_dist, 0.5);
  nh_.param<float>(icp_ns + "fitness_eps", cfg.icp.fitness_eps, 0.0005);
  nh_.param<float>(icp_ns + "transformation_eps", cfg.icp.transf_eps, 0.0001);

  cfg.output_dir = output_dir;
  cfg.icp.output_dir = output_dir;

  ROS_DEBUG_STREAM("Parameters loaded from rosparam server");

  return cfg;
}

void MapsIntegratorNode::IntegrateMaps(const std::string& map1_filepath, const std::string& map2_filepath) {
  auto config = GetConfigFromRosParams();

  auto map1 = LoadOcTreeFromFile(map1_filepath);
  auto map2 = LoadOcTreeFromFile(map2_filepath);

  MapsIntegrator maps_integrator(map1, map2, config);
  maps_integrator.EstimateTransformation();

  ROS_INFO("Maps transformation estimation finished");
}

}