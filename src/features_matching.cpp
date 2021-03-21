#include <features_matching.h>

#include <chrono>

#include <ros/console.h>

#include <pcl/conversions.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/features/board.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/registration/transformation_validation_euclidean.h>

#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <octomap_tools/maps_integrator_visualizer.h>

#include <thread_pool.h>
#include <conversions.h>
#include <validation.h>
#include <model_decomposition.h>


typedef pcl::ReferenceFrame RFType;
typedef pcl::Normal NormalType;

using namespace pcl;
using namespace pcl::registration;

namespace octomap_tools {

FeaturesMatching::FeaturesMatching(const Config& config, PointCloudPtr& scene, PointCloudPtr& model) :
  scene_cloud_(scene),
  model_cloud_(model),
  cfg_(config) {
}

FeaturesMatching::Result FeaturesMatching::DivideModelAndAlign(PointCloud& best_model) {
  auto start = std::chrono::high_resolution_clock::now();

  // Calculate scene descriptors
  auto scene = std::make_shared<FeatureCloud>(scene_cloud_, cfg_.feature_cloud);
  scene->ProcessInput();

  // Divide model into blocks
  Eigen::Vector4f map_min, map_max;
  pcl::getMinMax3D (*model_cloud_, map_min, map_max);
  std::vector<Rectangle> blocks = RectangularModelDecomposition(
    map_min, map_max, cfg_.cell_size_x, cfg_.cell_size_y);

  ROS_DEBUG_STREAM("Starting features matching alignment. There are " << blocks.size() << " blocks.");

  // Initialize thread pool and start workers
  thread_pool::ThreadPool thread_pool;
  std::vector<std::future<FeaturesMatching::ThreadResult>> thread_futures;
  for (std::uint32_t i = 0; i < blocks.size(); ++i) {
    thread_futures.emplace_back(thread_pool.Submit(
        AlignmentThread, i, std::ref(model_cloud_), std::ref(blocks[i]), std::ref(scene), std::ref(cfg_)));
  }

  // Wait till all workers finish
  std::vector<FeaturesMatching::ThreadResult> results;
  for (auto& it: thread_futures) {
    it.wait();
    results.emplace_back(it.get());
  }

  // Find best result from all threads results
  ThreadResult ia_result = FindBestAlignment(results);

  // Crop model to get the best matched block
  Point pmin, pmax;
  pcl::getMinMax3D(*model_cloud_, pmin, pmax);
  Eigen::Vector4f model_min = {blocks[ia_result.thread_num].min(0), blocks[ia_result.thread_num].min(1), pmin.z, 1};
  Eigen::Vector4f model_max = {blocks[ia_result.thread_num].max(0), blocks[ia_result.thread_num].max(1), pmax.z, 1};
  pcl::CropBox<Point> box_filter;
  box_filter.setInputCloud(model_cloud_);
  box_filter.setMin(model_min);
  box_filter.setMax(model_max);
  box_filter.filter(best_model);

  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);

  return FeaturesMatching::Result {ia_result.fitness_score, ia_result.transformation, static_cast<float>(diff.count())};
}

FeaturesMatching::ThreadResult FeaturesMatching::AlignmentThread(int nr,
                                                                PointCloudPtr& full_model,
                                                                Rectangle block,
                                                                const FeatureCloudPtr& scene,
                                                                FeaturesMatching::Config& cfg) {
  Point pmin, pmax;
  pcl::getMinMax3D(*full_model, pmin, pmax);
  Eigen::Vector4f model_min = {block.min(0), block.min(1), pmin.z, 1};
  Eigen::Vector4f model_max = {block.max(0), block.max(1), pmax.z, 1};

  pcl::CropBox<Point> box_filter;
  box_filter.setInputCloud(full_model);
  box_filter.setMin(model_min);
  box_filter.setMax(model_max);
  PointCloudPtr filtered_model (new PointCloud);
  box_filter.filter(*filtered_model);

  if (filtered_model->size() < cfg.model_size_thresh_) {
    ROS_WARN_STREAM("T" << nr << ": Model is too small (size: " << filtered_model->size() << ")");
    return FeaturesMatching::ThreadResult ();
  }

  auto model = std::make_shared<FeatureCloud>(filtered_model, cfg.feature_cloud);
  model->ExtractKeypoints();
  if (model->GetKeypoints()->size() < cfg.keypoints_thresh_) {
    ROS_WARN_STREAM("T" << nr << ": Not enough keypoints: " << model->GetKeypoints()->size());
    return FeaturesMatching::ThreadResult ();
  }
  model->ComputeSurfaceNormals();
  model->ComputeDescriptors();

  auto result = FeaturesMatching::Align(nr, cfg, model, scene);
  return FeaturesMatching::ThreadResult {true, nr, result.fitness_score, result.transformation};
}

FeaturesMatching::Result FeaturesMatching::Align(int nr,
                                                 FeaturesMatching::Config& cfg,
                                                 const FeatureCloudPtr& model,
                                                 const FeatureCloudPtr& scene) {
  Point ppmin, ppmax;
  pcl::getMinMax3D(*(model->GetPointCloud()), ppmin, ppmax);
  std::cout << "Task " << nr << ": Align template with " << model->GetKeypoints()->size() << " keypoints, "
            << std::setprecision(2) << std::fixed
            << "Pmin: (" << ppmin.x << ", " << ppmin.y << ", " << ppmin.z << ")  "
            << "Pmax: (" << ppmax.x << ", " << ppmax.y << ", " << ppmax.z << ")\n"
            << "  Scene size: " << scene->GetPointCloud()->size()
            << " keypoints: " << scene->GetKeypoints()->size()
            << " descriptors: " << scene->GetDescriptors()->size()
            << "\n"
            << "  Model size: " << model->GetPointCloud()->size()
            << " keypoints: " << model->GetKeypoints()->size()
            << " descriptors: " << model->GetDescriptors()->size()
            << "\n";

  auto start = std::chrono::high_resolution_clock::now();
  FeaturesMatching::Result result;
  pcl::CorrespondencesPtr features_correspondences (new pcl::Correspondences);

  if (cfg.method == AlignmentMethod::KdTreeSearch) {
    // KdTreeBasedAlignment aligner(cfg.)


  }
  else if (cfg.method == AlignmentMethod::SampleConsensus) {
    // Align feature clouds with Sample Consensus Initial Alignment
    SampleConsensusInitialAlignmentMod<Point, Point, FeatureCloud::DescriptorType> sac;
    sac.setMinSampleDistance(cfg.min_sample_distance);
    sac.setMaxCorrespondenceDistance(cfg.max_correspondence_distance);
    sac.setMaximumIterations(cfg.nr_iterations);
    sac.setNumberOfSamples(3);

    sac.setInputSource(model->GetKeypoints());
    sac.setSourceFeatures(model->GetDescriptors());
    sac.setInputTarget(scene->GetKeypoints());
    sac.setTargetFeatures(scene->GetDescriptors());

    PointCloud dummy_output;
    sac.alignMod(dummy_output);

    double fs = sac.getFitnessScore(cfg.fitness_score_dist);

    std::cout << "Converged?: " << sac.hasConverged()
      << std::setprecision(6) << std::fixed
      << "\nFitness score distance: " << cfg.fitness_score_dist << "\n"
      << "Standard fitness score: " << fs << "\n";

    pcl::registration::TransformationValidationEuclidean<Point, Point> validator;
    validator.setMaxRange(0.10);

    double score = validator.validateTransformation(model->GetKeypoints(), scene->GetKeypoints(),
                                          sac.getFinalTransformation());

    ROS_DEBUG_STREAM("Score: " << score);

    result.fitness_score = static_cast<float>(sac.getFitnessScore(cfg.fitness_score_dist));
    result.transformation = sac.getFinalTransformation();

    // features_correspondences = FindFeaturesCorrespondencesWithKdTree(model->GetDescriptors(), scene->GetDescriptors());
    pcl::CorrespondencesPtr correspondences2 = sac.getCorrespondences();

    std::cout << "\nCorrespondences n=" << features_correspondences->size() << "\n";
    std::cout << "\nCorrespondences2 n=" << features_correspondences->size() << "\n";
    std::cout << "\nFitnessScore1 corr1: " << calcFitnessScore1(features_correspondences) << "\n";
    std::cout << "\nFitnessScore1 corr2: " << calcFitnessScore1(correspondences2) << "\n";
    std::cout << "\nFitnessScore2 corr1: " << calcFitnessScore2(features_correspondences) << "\n";
    std::cout << "\nFitnessScore2 corr2: " << calcFitnessScore2(correspondences2) << "\n";
    std::cout << "\nFitnessScore3 corr1: " << calcFitnessScore3(features_correspondences) << "\n";
    std::cout << "\nFitnessScore3 corr2: " << calcFitnessScore3(correspondences2) << "\n";

  }
  else if (cfg.method == AlignmentMethod::GeometryConsistencyClustering) {
    float cg_size(0.01f);
    float cg_thresh(5.0f);

    // Find correspondences with KdTree
    // features_correspondences = FindFeaturesCorrespondencesWithKdTree(model->GetDescriptors(), scene->GetDescriptors());

    // Clustering
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
    gc_clusterer.setGCSize(cg_size);
    gc_clusterer.setGCThreshold (cg_thresh);

    gc_clusterer.setInputCloud(model->GetKeypoints());
    gc_clusterer.setSceneCloud(scene->GetKeypoints());
    gc_clusterer.setModelSceneCorrespondences (features_correspondences);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize(rototranslations, clustered_corrs);

    result.fitness_score = 0;
    if (rototranslations.size() == 1) {
      result.transformation = rototranslations[0];
    }
    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (size_t i = 0; i < rototranslations.size (); ++i) {
      std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
      std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;
    }
  }
  else if (cfg.method == AlignmentMethod::Hough3DClustering) {
    float rf_rad(0.015f);
    float cg_size(0.01f);
    float cg_thresh(5.0f);

    // Find correspondences with KdTree
    // features_correspondences = FindFeaturesCorrespondencesWithKdTree(model->GetDescriptors(), scene->GetDescriptors());

    // Clustering
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  Compute (Keypoints) Reference Frames only for Hough
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<Point, NormalType, RFType> rf_est;
    rf_est.setFindHoles(true);
    rf_est.setRadiusSearch(rf_rad);

    rf_est.setInputCloud(model->GetKeypoints());
    rf_est.setInputNormals(model->GetSurfaceNormals());
    rf_est.setSearchSurface(model->GetPointCloud());
    rf_est.compute(*model_rf);

    rf_est.setInputCloud(scene->GetKeypoints());
    rf_est.setInputNormals(scene->GetSurfaceNormals());
    rf_est.setSearchSurface(scene->GetPointCloud());
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<Point, Point, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size);
    clusterer.setHoughThreshold (cg_thresh);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model->GetKeypoints());
    clusterer.setInputRf(model_rf);
    clusterer.setSceneCloud (scene->GetKeypoints());
    clusterer.setSceneRf(scene_rf);
    clusterer.setModelSceneCorrespondences(features_correspondences);

    //clusterer.cluster(clustered_corrs);
    clusterer.recognize(rototranslations, clustered_corrs);

    ROS_DEBUG_STREAM("Model instances found: " << rototranslations.size());
    for (size_t i = 0; i < rototranslations.size (); ++i) {
      ROS_DEBUG_STREAM("    Instance " << i + 1 << ":");
      ROS_DEBUG_STREAM("        Correspondences belonging to this instance: " << clustered_corrs[i].size());
    }
  }

  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);
  ROS_DEBUG_STREAM("Task " << nr <<  ": Clouds aligned with score: "
    << result.fitness_score << " in " << diff.count() << " ms." << std::endl);

  result.correspondences = features_correspondences;

  if (cfg.show_visualizer || cfg.output_to_file) {
    MapsIntegratorVisualizer visualizer(
      { cfg.show_visualizer, cfg.output_to_file, cfg.output_dir + "feature_matching.png" });
    visualizer.VisualizeFeatureMatching(scene, model, result.transformation, result.correspondences);
  }

  return result;
}



FeaturesMatching::ThreadResult FeaturesMatching::FindBestAlignment(const std::vector<FeaturesMatching::ThreadResult>& results) {
  int block_nr = 0;
  float lowest_score = std::numeric_limits<float>::infinity();
  Eigen::Matrix4f transformation;
  for (auto it : results) {
    if (it.valid) {
      if (it.fitness_score < lowest_score) {
        lowest_score = it.fitness_score;
        block_nr = static_cast<int>(it.thread_num);
        transformation = it.transformation;
      }
    }
  }
  return ThreadResult{true, block_nr, lowest_score, transformation};
}

}