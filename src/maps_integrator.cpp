#include "octomap_tools/maps_integrator.h"

#include <ctime>
#include <cstdlib>

#include <iostream>
#include <iomanip>
#include <stdexcept>

#include <ros/console.h>

#include <pcl/point_cloud.h>

#include <common/transformations.h>
#include <alignment/features_matching.h>
#include <alignment/ndt_alignment.h>
#include <common/conversions.h>
#include <common/math.h>
#include <octomap_tools/octomap_io.h>
#include <common/utils.h>

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

  // Global alignment
  PointCloud::Ptr best_model(new PointCloud);

  if (cfg_.enable_global_alignment)
  {
    if (cfg_.global_alignment_method == GlobalAlignment::Method::FeatureMatching)
    {
      PCL_INFO("\nUsed Feature Matching Method\n");

      FeaturesMatching features_matching(cfg_.template_alignment, scene_, model_);

      if (cfg_.template_alignment.divide_model) {
        result_.ia = features_matching.DivideModelAndAlign(*best_model);
      }
      else {
        result_.ia = features_matching.align();
        best_model = model_;
      }
      result_.fitness_score1 = result_.ia.fitness_score1;
      result_.fitness_score2 = result_.ia.fitness_score2;
      result_.fitness_score3 = result_.ia.fitness_score3;
      result_.transformation = result_.ia.transformation;
      result_.transf_estimation_time_ms = result_.ia.processing_time_ms;
    }
    else if (cfg_.global_alignment_method == GlobalAlignment::Method::NDT) {
      PCL_INFO("\nUsed NDT Method\n");
      NdtAlignment ndt(cfg_.ndt_alignment, scene_, model_);
      auto result = ndt.Align();
      best_model = model_;
      result_.transformation = result.transformation;
      result_.transf_estimation_time_ms = result.processing_time_ms;
    }

    PCL_INFO("\nIA:");
    PCL_INFO("\n  fitness score: %.3f", result_.fitness_score1);
    PCL_INFO("\n  time: %.1f ms", result_.transf_estimation_time_ms);
    PCL_INFO("\n  transformation:\n%s", transfMatrixToXyzRpyString(result_.transformation, "    ").c_str());
  }
  else {
      best_model = model_;
  }

  // ICP correction
  if (cfg_.icp_correction) {
    try {
      PointCloud::Ptr icp_model(new PointCloud);

      if (cfg_.enable_global_alignment) {
        pcl::transformPointCloud(*best_model, *icp_model, result_.ia.transformation);

        ICP icp(scene_, icp_model, cfg_.icp);
        result_.icp = icp.Align();

        if (result_.icp.fitness_score1 < result_.ia.fitness_score1) {
          result_.fitness_score1 = result_.icp.fitness_score1;
          result_.fitness_score2 = result_.icp.fitness_score2;
          result_.fitness_score3 = result_.icp.fitness_score3;
          result_.transformation = result_.icp.transformation * result_.ia.transformation;
        }
      }
      else {
        ICP icp(scene_, model_, cfg_.icp);
        result_.icp = icp.Align();

        result_.fitness_score1 = result_.icp.fitness_score1;
        result_.fitness_score2 = result_.icp.fitness_score2;
        result_.fitness_score3 = result_.icp.fitness_score3;
        result_.transformation = result_.icp.transformation;
      }

      PCL_INFO("\nICP:");
      PCL_INFO("\n  fitness score: %.3f", result_.icp.fitness_score1);
      PCL_INFO("\n  time: %.1f ms", result_.icp.processing_time_ms);
      PCL_INFO("\n  transformation:\n%s", transfMatrixToXyzRpyString(result_.icp.transformation, "    ").c_str());
    }
    catch (std::exception& e) {
      PCL_ERROR("\nICP exception: %s", e.what());
    }
  }

  // Create result
  result_.transf_estimation_time_ms = (duration_cast<milliseconds>(high_resolution_clock::now() - start)).count();
  pcl::getMinMax3D(*best_model, result_.model_min, result_.model_max);

  PCL_INFO("\nFinal result:");
  PCL_INFO("\n  fitness score: %.3f", result_.fitness_score1);
  PCL_INFO("\n  time: %.1f s", result_.transf_estimation_time_ms / 1000.0);
  PCL_INFO("\n  transformation:\n%s\n", transfMatrixToXyzRpyString(result_.transformation, "    ").c_str());

  if (cfg_.output_to_file) {
    DumpConfigAndResultsToFile();
  }
  if (cfg_.show_visualizer || cfg_.output_to_file) {
    {
      MapsIntegratorVisualizer visualizer(
        { cfg_.show_visualizer, cfg_.output_to_file, cfg_.output_dir + "matching.png" });
      visualizer.VisualizeFeatureMatchingWithDividedModel(
        scene_, best_model, model_, result_.transformation, spiral_blocks_);
    }
    {
      PointCloudPtr transformed_model (new PointCloud);
      pcl::transformPointCloud(*model_, *transformed_model, result_.transformation);
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
  std::ofstream params(params_file, std::ios_base::app);
  params << cfg_.toString();

  std::string result_file = basename + "_result.txt";
  std::ofstream file(result_file, std::ios_base::app);
  file << OcTreeInfoToString(*scene_tree_, "scene_octree");
  file << OcTreeInfoToString(*model_tree_, "model_octree");
  file << result_.toString();
  return basename;
}

std::string MapsIntegrator::Result::toString() {
  std::stringstream ss;
  // Add octrees size
  ss << "maps_integration:\n";
  ss << "  transf_estimation_time_ms: " << transf_estimation_time_ms << "\n";
  ss << "  octree_transformation_time_ms: " << octree_transformation_time_ms << "\n";
  ss << "  octrees_merge_time_ms: " << octrees_merge_time_ms << "\n";
  ss << "global_alignment:\n";
  ss << "  transformation:\n" << transfMatrixToXyzRpyString(ia.transformation, "    ");
  ss << "  fitness_score1: " << ia.fitness_score1 << "\n";
  ss << "  fitness_score2: " << ia.fitness_score2 << "\n";
  ss << "  fitness_score3: " << ia.fitness_score3 << "\n";
  ss << "  time_ms: " << ia.processing_time_ms << "\n";
  ss << "  best_model_limits:\n";
  ss << "    x: [" << model_min.x << ", " << model_max.x << "]\n";
  ss << "    y: [" << model_min.y << ", " << model_max.y << "]\n";
  // ss << "  correspondences_num: " << ia.correspondences.size() << "\n";
  ss << "local_alignement:\n";
  ss << "  transformation:\n" << transfMatrixToXyzRpyString(icp.transformation, "    ");
  ss << "  fitness_score: " << icp.fitness_score1 << "\n";
  ss << "  fitness_score2: " << icp.fitness_score2 << "\n";
  ss << "  fitness_score3: " << icp.fitness_score3 << "\n";
  ss << "  time_ms: " << icp.processing_time_ms << "\n";
  ss << "final:\n";
  ss << "  transformation:\n" << transfMatrixToXyzRpyString(transformation, "    ");
  ss << "  fitness_score1: " << fitness_score1 << "\n";
  ss << "  fitness_score2: " << fitness_score2 << "\n";
  ss << "  fitness_score3: " << fitness_score3 << "\n";
  return ss.str();
}

std::string MapsIntegrator::Config::toString() {
  std::stringstream ss;
  // Global alignment
  ss << "enable_global_alignment: " << "1" << "\n";
  ss << "global_alignment: " << "\n";
  ss << "  method: ";
  if (template_alignment.method == FeaturesMatching::AlignmentMethodType::SampleConsensus)
    ss << "sac\n";
  if (template_alignment.method == FeaturesMatching::AlignmentMethodType::GeometryConsistencyClustering)
    ss << "gcc\n";

  ss << "  feature_cloud:\n";
  ss << "    descriptors_radius: "  << template_alignment.feature_cloud.descriptors_radius << "\n";
  ss << "    downsampling_radius: " << template_alignment.feature_cloud.downsampling_radius << "\n";
  ss << "    normal_radius: "       << template_alignment.feature_cloud.normal_radius << "\n";
  ss << "    keypoints_method: ";
  if (template_alignment.feature_cloud.keypoints_method == FeatureCloud::KeypointsExtractionMethod::Iss3d)
    ss << "iss3d\n";
  if (template_alignment.feature_cloud.keypoints_method == FeatureCloud::KeypointsExtractionMethod::Uniform)
    ss << "uniform\n";

  ss << "    iss3d:\n";
  ss << "      min_neighbours: " << template_alignment.feature_cloud.iss_min_neighbours << "\n";
  ss << "      iss_salient_radius: " << template_alignment.feature_cloud.iss_salient_radius << "\n";
  ss << "      iss_non_max_radius: " << template_alignment.feature_cloud.iss_non_max_radius << "\n";
  ss << "      iss_num_of_threads: " << template_alignment.feature_cloud.iss_num_of_threads << "\n";

  ss << "  divide_model: " << template_alignment.divide_model << "\n";
  ss << "  block_size: " << template_alignment.cell_size_x << "\n";
  ss << "  min_model_size: " << template_alignment.model_size_thresh_ << "\n";
  ss << "  min_keypoints_num: " << template_alignment.keypoints_thresh_ << "\n";

  ss << "  sample_consensus:\n";
  ss << "    fitness_score_distance: " << template_alignment.sac.fitness_score_dist << "\n";
  ss << "    iterations_num: " << template_alignment.sac.nr_iterations << "\n";
  ss << "    max_corr_dist: " << template_alignment.sac.max_correspondence_distance << "\n";
  ss << "    min_sample_dist: " << template_alignment.sac.min_sample_distance << "\n";
  ss << "    modified_version: " << template_alignment.sac.modified_version << "\n";


  // Local alignment
  ss << "enable_local_alignment: " << icp_correction << "\n";
  ss << "local_alignment:\n";
  ss << "  scene_inflation_dist: " << icp.scene_inflation_dist << "\n";
  ss << "  icp:\n";
  ss << "    max_iter: " << icp.max_iter << "\n";
  ss << "    max_nn_dist: " << icp.max_nn_dist << "\n";
  ss << "    fitness_eps: " << icp.fitness_eps << "\n";
  ss << "    fitness_score_dist: " << icp.fitness_score_dist << "\n";
  ss << "    transf_eps: " << icp.transf_eps << "\n";
  ss << "    crop_scene: " << icp.crop_scene << "\n";

  return ss.str();
}

}
