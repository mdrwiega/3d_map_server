#include <vector>
#include <exception>

#include <ros/init.h>
#include <ros/param.h>
#include <ros/package.h>
#include <ros/node_handle.h>
#include <ros/console.h>

#include <octomap_tools/utils.h>
#include <octomap_tools/octomap_io.h>
#include <octomap_tools/maps_integrator.h>

using namespace octomap_tools;

class MapsIntegratorNode {
 private:
  ros::NodeHandle nh_;

 public:
  MapsIntegrator::Config GetConfigFromRosParams() {
    MapsIntegrator::Config cfg;

    // Common
    std::string output_dir;
    nh_.param<std::string>("output_dir", output_dir, "");
    bool output_to_file;
    nh_.param<bool>("output_to_file", output_to_file, true);
    bool show_visualizer;
    nh_.param<bool>("show_visualizer", show_visualizer, false);

    // Main parameters
    cfg.show_visualizer = show_visualizer;

    // nh_.param<std::string>("global_alignment/method", cfg.method, "feature_matching");
    nh_.param<bool>("global_alignment/divide_model",cfg.template_alignment.divide_model, true);
    nh_.param<float>("global_alignment/cell_size_x", cfg.template_alignment.cell_size_x, 3.0);
    nh_.param<float>("global_alignment/cell_size_y", cfg.template_alignment.cell_size_y, 3.0);

    // Feature matching
    std::string fm_ns = "global_alignment/feature_matching/";
    auto& fm = cfg.template_alignment;
    nh_.param<int>(fm_ns + "min_model_size", fm.model_size_thresh_, 400);
    nh_.param<int>(fm_ns + "min_keypoints_num", fm.keypoints_thresh_, 40);
    nh_.param<float>(fm_ns + "normal_radius", fm.feature_cloud.normal_radius, 10.0);
    nh_.param<float>(fm_ns + "downsampling_radius", fm.feature_cloud.downsampling_radius, 0.15);
    nh_.param<float>(fm_ns + "descriptors_radius", fm.feature_cloud.descriptors_radius, 1.0);

    cfg.template_alignment.output_dir = output_dir;
    cfg.template_alignment.show_visualizer = show_visualizer;
    cfg.template_alignment.output_to_file = output_to_file;

    // Sample Consensus
    std::string sac_ns = "global_alignment/feature_matching/sample_consensus/";
    // nh_.param<std::string>("global_alignment/feature_matching/method",
    //   cfg.template_alignment.method, "sample_consensus");
    nh_.param<int>(sac_ns + "iterations_num", cfg.template_alignment.nr_iterations, 1000);
    nh_.param<float>(sac_ns + "min_sample_distance", cfg.template_alignment.min_sample_distance, 0.2);
    nh_.param<float>(sac_ns + "max_correspondence_distance", cfg.template_alignment.max_correspondence_distance, 100.0);
    nh_.param<float>(sac_ns + "fitness_score_distance", cfg.template_alignment.fitness_score_dist, 0.5);

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

  void IntegrateMaps(const std::string& map1_filepath, const std::string& map2_filepath) {
    auto config = GetConfigFromRosParams();

    auto map1 = LoadOcTreeFromFile(map1_filepath);
    auto map2 = LoadOcTreeFromFile(map2_filepath);

    MapsIntegrator maps_integrator(map1, map2, config);
    maps_integrator.EstimateTransformation();
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "maps_integrator");

  const std::string map1_filepath = argv[1];
  const std::string map2_filepath = argv[2];

  if (!exists(map1_filepath)) {
    ROS_ERROR_STREAM("File doesn't exist in path: " << map1_filepath << std::endl);
    exit(1);
  }
  if (!exists(map2_filepath)) {
    ROS_ERROR_STREAM("File doesn't exist in path: " << map2_filepath << std::endl);
    exit(1);
  }

  ROS_INFO_STREAM("Maps Integrator node started");

  try {
    MapsIntegratorNode maps_integrator;
    maps_integrator.IntegrateMaps(map1_filepath, map2_filepath);
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("Exception" << ex.what());
  }

  return 0;
}