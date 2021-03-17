#include "octomap_tools/maps_integrator.h"

#include <ctime>
#include <cstdlib>

#include <iostream>
#include <iomanip>
#include <stdexcept>

#include <ros/console.h>

#include <pcl/point_cloud.h>

#include <octomap_tools/transformations.h>
#include <octomap_tools/features_matching.h>
#include <octomap_tools/conversions.h>
#include <octomap_tools/math.h>
#include <octomap_tools/octomap_io.h>

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;
using std::chrono::duration_cast;

namespace octomap_tools {

MapsIntegrator::MapsIntegrator(const OcTreePtr& scene_tree, const OcTreePtr& model_tree, const Config& config) :
  model_tree_(model_tree),
  scene_tree_(scene_tree),
  cfg_(config) {

  model_ = OcTreeToPointCloud(*model_tree_);
  scene_ = OcTreeToPointCloud(*scene_tree_);

  if (model_->size() < cfg_.template_alignment.model_size_thresh_) {
    throw (std::runtime_error(std::string(__func__) + ": Model size is too small"));
  }
  if (scene_->size() < cfg_.template_alignment.model_size_thresh_) {
    throw (std::runtime_error(std::string(__func__) + ": Scene size is too small"));
  }
}

MapsIntegrator::Result MapsIntegrator::EstimateTransformation() {
  auto start = high_resolution_clock::now();

  // Features based initial alignment
  PointCloud::Ptr best_model(new PointCloud);
  FeaturesMatching features_matching(cfg_.template_alignment, scene_, model_);

  if (cfg_.template_alignment.divide_model) {
    result_.ia = features_matching.DivideModelAndAlign(*best_model);
  }

  ROS_DEBUG_STREAM(std::setprecision(6) << "IA fitness score: " << result_.ia.fitness_score);
  ROS_DEBUG_STREAM(std::setprecision(1) << "IA time: " << result_.ia.processing_time_ms << " ms");
  ROS_DEBUG_STREAM("IA transformation:" << transfMatrixToXyzRpyString(result_.ia.transformation));

  // ICP correction
  if (cfg_.icp_correction) {
    PointCloud::Ptr icp_model(new PointCloud);
    pcl::transformPointCloud(*best_model, *icp_model, result_.ia.transformation);
    ICP icp(scene_, icp_model, cfg_.icp);
    result_.icp = icp.Align();

    if (result_.icp.fitness_score < result_.ia.fitness_score) {
      result_.ia.fitness_score = result_.icp.fitness_score;
      result_.ia.transformation = result_.icp.transformation * result_.ia.transformation;
    }

    ROS_DEBUG_STREAM(std::setprecision(6) << "ICP fitness score: " << result_.icp.fitness_score);
    ROS_DEBUG_STREAM(std::setprecision(1) << "ICP time: " << result_.icp.processing_time_ms << " ms");
    ROS_DEBUG_STREAM("ICP transformation:" << transfMatrixToXyzRpyString(result_.ia.transformation));
  }

  // Create result
  result_.transf_estimation_time_ms = (duration_cast<milliseconds>(high_resolution_clock::now() - start)).count();
  result_.fitness_score = result_.ia.fitness_score;
  pcl::getMinMax3D(*best_model, result_.model_min, result_.model_max);
  result_.transformation = result_.ia.transformation;

  result_.PrintResult();

  if (cfg_.output_to_file) {
    DumpConfigAndResultsToFile();
  }
  if (cfg_.show_visualizer || cfg_.output_to_file) {
    {
      MapsIntegratorVisualizer visualizer(
        { cfg_.show_visualizer, cfg_.output_to_file, cfg_.output_dir + "matching.png" });
      visualizer.VisualizeFeatureMatchingWithDividedModel(
        scene_, best_model, model_, result_.ia.transformation, spiral_blocks_);
    }
    {
      PointCloudPtr transformed_model (new PointCloud);
      pcl::transformPointCloud(*model_, *transformed_model, result_.ia.transformation);
      MapsIntegratorVisualizer visualizer(
        { cfg_.show_visualizer, cfg_.output_to_file, cfg_.output_dir + "matching_2clouds.png" });
      visualizer.VisualizeClouds(scene_, transformed_model);
    }
  }
  return result_;
}

OcTreePtr MapsIntegrator::Merge(const Eigen::Matrix4f& transformation, bool save_to_file) {
  auto start = high_resolution_clock::now();

  // Transform map
  auto tree_model_transformed = FastOcTreeTransform(*model_tree_, transformation);

  auto diff = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - start);
  ROS_DEBUG_STREAM("Map transformed in " << diff.count() << " ms.");

  start = high_resolution_clock::now();

  // Merge maps which are already in the same coordination system
  auto merged_tree = FastSumOctrees(*tree_model_transformed, *scene_tree_);

  diff = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - start);
  ROS_DEBUG_STREAM("Maps merged in " << diff.count() << " ms.");

  if (save_to_file) {
    SaveOcTreeToFile(*merged_tree, cfg_.output_dir + "merged_tree.ot");
  }
  return merged_tree;
}

OcTreePtr MapsIntegrator::Merge(bool save_to_file) {
  EstimateTransformation();
  return Merge(result_.transformation, save_to_file);
}

std::string MapsIntegrator::DumpConfigAndResultsToFile(const std::string& filename) {
  std::string basename = cfg_.output_dir;
  if (!filename.empty()) {
    basename = filename;
  }

  std::string params_file = basename + "_params.yaml";
  std::system(("rosparam dump " + params_file).c_str());

  std::string result_file = basename + "_result.txt";
  std::ofstream file(result_file, std::ios_base::app);
  file << result_.toString();
  return basename;
}

std::string MapsIntegrator::Result::toString() {
  std::stringstream ss;
  // Add octrees size
  ss << "model_min_x: " << model_min.x << "\n";
  ss << "model_min_y: " << model_min.y << "\n";
  ss << "model_max_x: " << model_max.x << "\n";
  ss << "model_max_y: " << model_max.y << "\n";
  ss << "fitness_score1: " << fitness_score << "\n";
  ss << "fitness_score2: " << fitness_score2 << "\n";
  ss << "fitness_score3: " << fitness_score3 << "\n";
  ss << "transf_estimation_time_ms: " << transf_estimation_time_ms << "\n";
  ss << "octree_transformation_time_ms: " << octree_transformation_time_ms << "\n";
  ss << "octrees_merge_time_ms: " << octrees_merge_time_ms << "\n";
  ss << "ia_transformation:\n" << transfMatrixToXyzRpyString(ia.transformation, "  ");
  ss << "icp_transformation:\n" << transfMatrixToXyzRpyString(icp.transformation, "  ");
  ss << "est_transformation:\n" << transfMatrixToXyzRpyString(transformation, "  ");
  return ss.str();
}

void MapsIntegrator::Result::PrintResult() {
  std::cout << "Final result:" << std::fixed << std::setprecision(1) << std::endl;
  std::cout << "Total time: " << (transf_estimation_time_ms + octree_transformation_time_ms + octrees_merge_time_ms) / 1000.0 << " s." << std::endl;
  std::cout << "Fitness score: " << std::setprecision(6) << fitness_score << std::endl;
  std::cout << "Transformation: " << transfMatrixToXyzRpyString(transformation) << std::endl;
}

std::string MapsIntegrator::Config::toString() {
  std::stringstream stream;
  auto header = getHeader();
  for (const auto& i : header) {
    if (&i != &header.back())
      stream << i << "; ";
    else
      stream << i << std::endl;
  }

  stream << template_alignment.model_size_thresh_ << "; " << fitness_score_thresh << "; " << template_alignment.keypoints_thresh_ << "; " << template_alignment.cell_size_x << "; " << template_alignment.cell_size_y << "; "
      << icp.max_iter << "; " << icp.max_nn_dist << "; " << icp.fitness_eps << "; " << icp.transf_eps << "; " << icp.scene_inflation_dist
      << "; " << template_alignment.feature_cloud.normal_radius << "; " << template_alignment.feature_cloud.downsampling_radius << "; " << template_alignment.feature_cloud.descriptors_radius
      << "; " << template_alignment.min_sample_distance << "; " << template_alignment.max_correspondence_distance
      << "; " << template_alignment.nr_iterations << std::endl;
  return stream.str();
}

std::vector<std::string> MapsIntegrator::Config::getHeader() {
  return  { "model_size_thresh", "fitness_score_thresh", "keypoints_thresh", "cell_size_x", "cell_size_y",
            "icp.max_iter", "icp.max_nn_dist", "icp.fitness_eps", "icp.transf_eps", "icp.scene_infl_dist",
            "fc.normal_radius", "fc.downsampling_radius", "fc.descriptors_radius", "ta.min_sample_dist", "ta.max_corr_dist",
            "ta.nr_iter" };
}

}
