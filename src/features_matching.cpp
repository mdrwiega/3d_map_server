#include <octomap_tools/features_matching.h>

#include <chrono>

#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/correspondence.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/features/board.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/recognition/cg/geometric_consistency.h>

#include <octomap_tools/thread_pool.h>

typedef pcl::ReferenceFrame RFType;
typedef pcl::Normal NormalType;

namespace octomap_tools {

FeaturesMatching::FeaturesMatching(const Config& config, PointCloudPtr& scene, PointCloudPtr& model) :
  scene_cloud_(scene),
  model_cloud_(model),
  cfg_(config) {
}

FeaturesMatching::Result FeaturesMatching::Align(int nr,
                                                 FeaturesMatching::Config& cfg,
                                                 const FeatureCloudPtr model,
                                                 const FeatureCloudPtr scene) {
  if (cfg.debug) {
    Point ppmin, ppmax;
    pcl::getMinMax3D(*(model->GetPointCloud()), ppmin, ppmax);
    std::cout << "Task " << nr << ": Align template with " << model->GetKeypoints()->size() << " keypoints, ";
    std::cout << std::setprecision(2) << std::fixed;
    std::cout << "Pmin: (" << ppmin.x << ", " << ppmin.y << ", " << ppmin.z << ")  ";
    std::cout << "Pmax: (" << ppmax.x << ", " << ppmax.y << ", " << ppmax.z << ")\n";
    std::cout << "  Scene size: " << scene->GetPointCloud()->size() << " NaNs: " << GetNumberOfNaNInPointCloud(*scene->GetPointCloud())
              << " keypoints: " << scene->GetKeypoints()->size() << " NaNs: " << GetNumberOfNaNInPointCloud(*scene->GetKeypoints())
              << " descriptors: " << scene->GetDescriptors()->size()
              << "\n"
              << "  Model size: " << model->GetPointCloud()->size() << " NaNs: " << GetNumberOfNaNInPointCloud(*model->GetPointCloud())
              << " keypoints: " << model->GetKeypoints()->size() << " NaNs: " << GetNumberOfNaNInPointCloud(*model->GetKeypoints())
              << " descriptors: " << model->GetDescriptors()->size()
              << "\n\n";
  }

  auto start = std::chrono::high_resolution_clock::now();

  FeaturesMatching::Result result;
  pcl::CorrespondencesPtr correspondences;

  if (cfg.method == AlignmentMethod::SampleConsensus) {
    // Align feature clouds with Sample Consensus Initial Alignment
    pcl::SampleConsensusInitialAlignment<Point, Point, FeatureCloud::DescriptorType> sac_ia;
    sac_ia.setMinSampleDistance(cfg.min_sample_distance);
    sac_ia.setMaxCorrespondenceDistance(cfg.max_correspondence_distance);
    sac_ia.setMaximumIterations(cfg.nr_iterations);
    sac_ia.setNumberOfSamples(3);

    sac_ia.setInputSource(model->GetKeypoints());
    sac_ia.setSourceFeatures(model->GetDescriptors());
    sac_ia.setInputTarget(scene->GetKeypoints());
    sac_ia.setTargetFeatures(scene->GetDescriptors());

    PointCloud dummy_output;
    sac_ia.align(dummy_output);
    result.fitness_score = static_cast<float>(sac_ia.getFitnessScore(cfg.fitness_score_dist));
    result.transformation = sac_ia.getFinalTransformation();
  }
  else if (cfg.method == AlignmentMethod::GeometryConsistencyClustering) {
    float cg_size_ (0.01f);
    float cg_thresh_ (5.0f);

    // Find correspondences with KdTree
    correspondences = FindCorrespondencesWithKdTree(model->GetDescriptors(), scene->GetDescriptors());

    // Clustering
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
    gc_clusterer.setGCSize(cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud(model->GetKeypoints());
    gc_clusterer.setSceneCloud(scene->GetKeypoints());
    gc_clusterer.setModelSceneCorrespondences (correspondences);

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
    float rf_rad_ (0.015f);
    float cg_size_ (0.01f);
    float cg_thresh_ (5.0f);

    // Find correspondences with KdTree
    correspondences = FindCorrespondencesWithKdTree(model->GetDescriptors(), scene->GetDescriptors());

    // Clustering
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    std::vector<pcl::Correspondences> clustered_corrs;

    //  Compute (Keypoints) Reference Frames only for Hough
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<Point, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model->GetKeypoints());
    rf_est.setInputNormals (model->GetSurfaceNormals());
    rf_est.setSearchSurface (model->GetPointCloud());
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene->GetKeypoints());
    rf_est.setInputNormals (scene->GetSurfaceNormals());
    rf_est.setSearchSurface (scene->GetPointCloud());
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<Point, Point, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model->GetKeypoints());
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene->GetKeypoints());
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (correspondences);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);

    std::cout << "Model instances found: " << rototranslations.size () << std::endl;
    for (size_t i = 0; i < rototranslations.size (); ++i) {
      std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
      std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;
    }
  }

  if (cfg.debug) {
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - start);
    std::cout << "Task " << nr <<  ": Clouds aligned with score: "
              << result.fitness_score << " in: " << diff.count() << " ms." << std::endl;
  }

  result.correspondences = correspondences;

  if (cfg.visualize) {
    Visualize(model, scene, result);
  }

  return result;
}

FeaturesMatching::ThreadResult threadFcn(int nr, PointCloudPtr& full_model, std::vector<Rectangle> blocks_,
                                         const FeatureCloudPtr& scene, FeaturesMatching::Config& cfg) {
  Point pmin, pmax;
  pcl::getMinMax3D(*full_model, pmin, pmax);
  Eigen::Vector4f model_min = {blocks_[nr].min(0), blocks_[nr].min(1), pmin.z, 1};
  Eigen::Vector4f model_max = {blocks_[nr].max(0), blocks_[nr].max(1), pmax.z, 1};

  pcl::CropBox<Point> boxFilter;
  boxFilter.setInputCloud(full_model);
  boxFilter.setMin(model_min);
  boxFilter.setMax(model_max);
  PointCloudPtr filtered_model (new PointCloud);
  boxFilter.filter(*filtered_model);

  if (filtered_model->size() < cfg.model_size_thresh_) {
    //    std::cout << "T" << nr << ": Model is too small (size: " << filtered_model->size() << ")\n";
    return FeaturesMatching::ThreadResult ();
  }

  auto model = std::make_shared<FeatureCloud>(filtered_model, cfg.feature_cloud);
  model->ExtractKeypoints();
  if (model->GetKeypoints()->size() < cfg.keypoints_thresh_) {
    //    std::cout << "T" << nr << ": Not enough keypoints: " << model->GetKeypoints()->size() << "\n";
    return FeaturesMatching::ThreadResult ();
  }
  model->ComputeSurfaceNormals();
  model->ComputeDescriptors();

  auto result = FeaturesMatching::Align(nr, cfg, model, scene);
  return FeaturesMatching::ThreadResult {true, nr, result.fitness_score, result.transformation};
}

FeaturesMatching::Result FeaturesMatching::initialAlignment(PointCloud& best_model) {
  auto start = std::chrono::high_resolution_clock::now();

  auto scene = std::make_shared<FeatureCloud>(scene_cloud_, cfg_.feature_cloud);
  scene->ProcessInput();
  std::vector<Rectangle> blocks_ = RectangularModelDecomposition(cfg_.cell_size_x, cfg_.cell_size_y);

  std::cout << "\nStarting features matching alignment. There are " << blocks_.size() << " blocks.\n";

  std::shared_ptr<thread_pool::ThreadPool> thread_pool = thread_pool::createThreadPool();
  std::vector<std::future<FeaturesMatching::ThreadResult>> thread_futures;
  for (std::uint32_t i = 0; i < blocks_.size(); ++i) {
    thread_futures.emplace_back(thread_pool->submit(
        threadFcn, i, std::ref(model_cloud_), std::ref(blocks_), std::ref(scene), std::ref(cfg_)));
  }

  std::vector<FeaturesMatching::ThreadResult> results;
  for (auto& it: thread_futures) {
    it.wait();
    results.emplace_back(it.get());
  }

  ThreadResult ia_result = findBestAlignment(results);

  // Get best model
  Point pmin, pmax;
  pcl::getMinMax3D(*model_cloud_, pmin, pmax);
  Eigen::Vector4f model_min = {blocks_[ia_result.thread_num].min(0), blocks_[ia_result.thread_num].min(1), pmin.z, 1};
  Eigen::Vector4f model_max = {blocks_[ia_result.thread_num].max(0), blocks_[ia_result.thread_num].max(1), pmax.z, 1};
  pcl::CropBox<Point> boxFilter;
  boxFilter.setInputCloud(model_cloud_);
  boxFilter.setMin(model_min);
  boxFilter.setMax(model_max);
  boxFilter.filter(best_model);

  auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start);

  return FeaturesMatching::Result {ia_result.fitness_score, ia_result.transformation, static_cast<float>(diff.count())};
}

pcl::CorrespondencesPtr FeaturesMatching::FindCorrespondencesWithKdTree(
    FeatureCloud::Descriptors::Ptr model_descriptors,
    FeatureCloud::Descriptors::Ptr scene_descriptors) {

  pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());

  pcl::KdTreeFLANN<FeatureCloud::DescriptorType> match_search;
  match_search.setInputCloud(model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor
  //  into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t i = 0; i < scene_descriptors->size (); ++i) {
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) { //skipping NaNs
      continue;
    }

    int found_neighs = match_search.nearestKSearch(scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);

    // add match only if the squared descriptor distance is
    // less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) {
      pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
      correspondences->push_back(corr);
    }
  }
  std::cout << "Correspondences found: " << correspondences->size () << std::endl;
  return correspondences;
}

std::vector<Rectangle> FeaturesMatching::RectangularModelDecomposition(float block_size_x, float block_size_y) {
  if (model_cloud_->size() <= 0) {
    throw std::runtime_error(std::string("Full model is not set!"));
  }

  Eigen::Vector4f map_min, map_max;
  pcl::getMinMax3D (*model_cloud_, map_min, map_max);
  Eigen::Vector2f rectangle_min (map_min.x(), map_min.y());
  Eigen::Vector2f rectangle_max (map_max.x(), map_max.y());
  Eigen::Vector2f step_xy (block_size_x, block_size_y);

  if (cfg_.debug) {
   std::cout << "Generation of cells for rectangle "
       << "min: (" << rectangle_min(0) << ", " << rectangle_min(1) << ")  "
       << "max: (" << rectangle_max(0) << ", " << rectangle_max(1) << ")\n";
  }
  return generateBlocksInSpiralOrder(rectangle_min, rectangle_max, step_xy);
}

FeaturesMatching::ThreadResult FeaturesMatching::findBestAlignment(const std::vector<FeaturesMatching::ThreadResult>& results) {
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

void FeaturesMatching::Visualize(
  const FeatureCloudPtr model, const FeatureCloudPtr scene, const FeaturesMatching::Result& result,
  bool show_keypoints, bool show_correspondences) {

  pcl::visualization::PCLVisualizer viewer ("Feature matching");
  viewer.setBackgroundColor(255, 255, 255);
  viewer.addCoordinateSystem(1.0);
  viewer.addPointCloud (scene->GetPointCloud(), "scene_cloud");

  pcl::PointCloud<Point>::Ptr off_scene_model (new pcl::PointCloud<Point> ());
  pcl::PointCloud<Point>::Ptr off_scene_model_keypoints (new pcl::PointCloud<Point> ());

  if (show_correspondences || show_keypoints) {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model->GetPointCloud(), *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model->GetKeypoints(), *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<Point> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints) {
    pcl::visualization::PointCloudColorHandlerCustom<Point> scene_keypoints_color_handler (scene->GetKeypoints(), 0, 0, 255);
    viewer.addPointCloud (scene->GetKeypoints(), scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<Point> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }

  pcl::PointCloud<Point>::Ptr rotated_model (new pcl::PointCloud<Point> ());
  pcl::transformPointCloud (*model->GetPointCloud(), *rotated_model, result.transformation);

  pcl::visualization::PointCloudColorHandlerCustom<Point> rotated_model_color_handler (rotated_model, 255, 0, 0);
  viewer.addPointCloud (rotated_model, rotated_model_color_handler, "instance");

  if (show_correspondences && result.correspondences != nullptr) {
    for (size_t j = 0; j < (*result.correspondences).size (); ++j) {
      std::stringstream ss_line;
      ss_line << "correspondence_line_" << j;
      Point& model_point = off_scene_model_keypoints->at((*result.correspondences)[j].index_query);
      Point& scene_point = scene->GetKeypoints()->at((*result.correspondences)[j].index_match);

      //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
      viewer.addLine<Point, Point> (model_point, scene_point, 0, 255, 0, ss_line.str ());
    }
  }

  while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
  }
}

}
