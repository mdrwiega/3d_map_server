#include "octomap_tools/maps_integrator.h"

#include <iostream>
#include <iomanip>
#include <ctime>

#include <pcl/point_cloud.h>

#include <octomap_tools/transformations.h>
#include <octomap_tools/features_matching.h>
#include <octomap_tools/conversions.h>
#include <octomap_tools/math.h>
#include <octomap_tools/octomap_io.h>
#include "utils/table_printer.h"

using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

namespace octomap_tools {

MapsIntegrator::MapsIntegrator(const OcTreePtr& scene_tree, const OcTreePtr& model_tree, const Config& config) :
  model_tree_(model_tree),
  scene_tree_(scene_tree),
  cfg_(config) {
  model_ = OcTreeToPointCloud(*model_tree_);
  scene_ = OcTreeToPointCloud(*scene_tree_);
}

MapsIntegrator::Result MapsIntegrator::EstimateTransformation() {
  auto start = high_resolution_clock::now();

  // Features based initial alignment
  PointCloud::Ptr best_model(new PointCloud);
  FeaturesMatching features_matching(cfg_.template_alignment, scene_, model_);
  FeaturesMatching::Result ia_result = features_matching.DivideModelAndAlign(*best_model);

  if (cfg_.debug) {
    std::cout << std::setprecision(6) << "\nIA fitness score: " << ia_result.fitness_score << "\n";
    std::cout << std::setprecision(1) << "IA time: " << ia_result.processing_time_ms << " ms\n";
    std::cout << "IA transformation:" << transfMatrixToXyzRpyString(ia_result.transformation) << "\n";
  }

  // ICP correction
  if (cfg_.icp_correction) {
      PointCloud::Ptr icp_model(new PointCloud);
      pcl::transformPointCloud(*best_model, *icp_model, ia_result.transformation);
      ICP icp(scene_, icp_model, cfg_.icp);
      ICP::Result icp_result = icp.Align();

      if (icp_result.fitness_score < ia_result.fitness_score) {
        ia_result.fitness_score = icp_result.fitness_score;
        ia_result.transformation = icp_result.transformation * ia_result.transformation;
      }

      if (cfg_.debug) {
        std::cout << std::setprecision(6) << "ICP fitness score: " << icp_result.fitness_score << "\n";
        std::cout << std::setprecision(1) << "ICP time: " << icp_result.processing_time_ms << " ms\n";
        std::cout << "ICP transformation:" << transfMatrixToXyzRpyString(ia_result.transformation) << "\n";
      }
  }

  // Create result
  result_ = Result();
  result_.time_ms = (std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - start)).count();
  result_.fitness_score = ia_result.fitness_score;
  pcl::getMinMax3D(*best_model, result_.model_min, result_.model_max);
  result_.transformation = ia_result.transformation;

  result_.PrintResult();

  if (cfg_.dump_to_file_) {
    DumpConfigAndResultsToFile();
  }
  if (cfg_.show_visualization_) {
    MapsIntegratorVisualizer visualizer({ true, cfg_.files_path_and_pattern + "matching.png" });
    visualizer.VisualizeFeatureMatchingWithDividedModel(
      scene_, best_model, model_, ia_result.transformation, spiral_blocks_);
  }
  if (cfg_.show_two_pointclouds) {
    PointCloudPtr transformed_model (new PointCloud);
    pcl::transformPointCloud(*model_, *transformed_model, ia_result.transformation);
    MapsIntegratorVisualizer visualizer({ true, cfg_.files_path_and_pattern + "matching_2clouds.png" });
    visualizer.VisualizeClouds(scene_, transformed_model);
  }
  return result_;
}

OcTreePtr MapsIntegrator::Merge(const Eigen::Matrix4f& transformation, bool save_to_file) {
  auto start = high_resolution_clock::now();

  // Transform map
  auto tree_model_transformed = FastOcTreeTransform(*model_tree_, transformation);

  if (cfg_.debug) {
    auto diff = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - start);
    std::cout << "Map transformed in " << diff.count() << " ms." << std::endl;
  }

  start = high_resolution_clock::now();

  // Merge maps which are already in the same coordination system
  auto merged_tree = FastSumOctrees(*tree_model_transformed, *scene_tree_);

  if (cfg_.debug) {
    auto diff = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - start);
    std::cout << "Maps merged in " << diff.count() << " ms." << std::endl;
  }

  if (save_to_file) {
    SaveOcTreeToFile(*merged_tree, cfg_.files_path_and_pattern + "merged_tree.ot");
  }
  return merged_tree;
}

OcTreePtr MapsIntegrator::Merge(bool save_to_file) {
  EstimateTransformation();
  return Merge(result_.transformation, save_to_file);
}

void MapsIntegrator::DumpConfigAndResultsToFile() {
  std::ofstream file(cfg_.files_path_and_pattern + "cfg_results.txt");
  file << cfg_.toTable();
  file << result_.toString();
}

std::string MapsIntegrator::Result::toString() {
  std::stringstream stream;
  md::TablePrinter tp(&stream);
  const std::string columns[] = {
      "time[ms]", "fitness_score", "model_min_x", "model_min_y", "model_max_x", "model_max_y"
  };

  for (const auto& i : columns) {
    tp.addColumn(i);
  }

  tp.printTitle("Feature matching results");
  tp.printHeader();
  tp << time_ms << fitness_score << model_min.x << model_min.y << model_max.x << model_max.y;
  tp.printFooter();
  return stream.str();
}

void MapsIntegrator::Result::PrintResult() {
  std::cout << "Final result:" << std::fixed << std::setprecision(1) << std::endl;
  std::cout << "Total processing time: " << time_ms / 1000.0 << " s." << std::endl;
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

std::string MapsIntegrator::Config::toTable() {
  std::stringstream stream;
  md::TablePrinter tp(&stream);

  for (const auto& i : getHeader()) {
    tp.addColumn(i);
  }

  tp.printTitle("Features matching parameters");
  tp.printHeader();

  tp
      << template_alignment.model_size_thresh_ << fitness_score_thresh << template_alignment.keypoints_thresh_ << template_alignment.cell_size_x << template_alignment.cell_size_y
      << icp.max_iter << icp.max_nn_dist << icp.fitness_eps << icp.transf_eps << icp.scene_inflation_dist
      << template_alignment.feature_cloud.normal_radius << template_alignment.feature_cloud.downsampling_radius << template_alignment.feature_cloud.descriptors_radius
      << template_alignment.min_sample_distance << template_alignment.max_correspondence_distance
      << template_alignment.nr_iterations;

  tp.printFooter();
  return stream.str();
}

}
