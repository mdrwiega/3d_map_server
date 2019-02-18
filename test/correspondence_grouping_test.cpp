/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>

#include "test_utils.h"
#include "octomap_integrator.h"
#include "md_utils/math/transformations.h"
#include "octree_transformations.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <utils/pointcloud_utils.h>

#include <pcl/filters/crop_box.h>



using namespace Eigen;
using namespace octomap_tools;
using namespace octomap;
using namespace md;
using namespace std::chrono;
#define CALC_CORRESPONDENCE 1
#define SHOW_IMAGES 0
#define SHOW_PCL 0
#define RUN_OCTOVIS 0
#define SAVE_CLOUDS_TO_FILES 0
#define LOAD_CLOUDS_FROM_FILES 0



TEST(IntegrateOctomaps, GenerateSpiralTraverse)
{
  Eigen::Vector2f rectangle_min (-4, -2);
  Eigen::Vector2f rectangle_max (2, 3);
  Eigen::Vector2f step_xy (1, 2);

  auto cells_seq = generateBlocksInSpiralOrder(rectangle_min, rectangle_max, step_xy);

  for (size_t i = 0; i < cells_seq.size(); ++i) {
    auto cell = cells_seq[i];
    std::cout << "Block nr: " << i << "  min: (" << cell.min(0) << ", " << cell.min(1) << ")  max: (" << cell.max(0) << ", " << cell.max(1) << ")\n";
  }
}

CGResultsSet estimateTfsByCorrespondenceClustering(CorrespondenceGroupingConfig& config,
                                                   PointCloudPtr& cloud_l, PointCloudPtr& cloud_r) {
  // Divide right map into cells
  Point cell_size(config.cell_size_x_, config.cell_size_y_, 0);
  CGResultsSet results;
  FeatureCloudPtr scene = FeatureCloudPtr (new FeatureCloud);
  scene->setInputCloud(cloud_l);

  Vector4f cloud_r2_min, cloud_r2_max;
  pcl::getMinMax3D (*cloud_r, cloud_r2_min, cloud_r2_max);
  Eigen::Vector2f rectangle_min (cloud_r2_min.x(), cloud_r2_min.y());
  Eigen::Vector2f rectangle_max (cloud_r2_max.x(), cloud_r2_max.y());
  Eigen::Vector2f step_xy (cell_size.x, cell_size.y);

  std::cout << "Generation of cells for rectangle min: (" << rectangle_min(0) << ", " << rectangle_min(1) << ")  max: (" << rectangle_max(0) << ", " << rectangle_max(1) << ")\n";
  auto cells_seq = generateBlocksInSpiralOrder(rectangle_min, rectangle_max, step_xy);

  for (size_t i = 0; i < cells_seq.size(); ++i) {
    auto cell = cells_seq[i];
    std::cout << "Block nr: " << i << "  min: (" << cell.min(0) << ", " << cell.min(1) << ")  max: (" << cell.max(0) << ", " << cell.max(1) << ")\n";
    Vector4f model_min = {cell.min(0), cell.min(1), cloud_r2_min(2), 1};
    Vector4f model_max = {cell.max(0), cell.max(1), cloud_r2_max(2), 1};

    pcl::CropBox<Point> boxFilter;
    boxFilter.setMin(model_min);
    boxFilter.setMax(model_max);
    boxFilter.setInputCloud(cloud_r);
    PointCloudPtr cloud_model (new PointCloud);
    boxFilter.filter(*cloud_model);

    if (cloud_model->size() < config.model_size_thresh_) {
      std::cout << "Model is too small (size= " << cloud_model->size() << "\n";
      continue;
    }

    FeatureCloudPtr model = FeatureCloudPtr (new FeatureCloud);
    model->setInputCloud(cloud_model);
    calc(scene, model, config, results);
  }
  return results;
}

TEST(IntegrateOctomaps, CorrespondenceGroupingPcl_Demo_PclVis)
{
  PointCloudPtr orig_cloud (new PointCloud);
  PointCloudPtr cropped_cloud (new PointCloud);

  PointCloudPtr cloud_l (new PointCloud);
  PointCloudPtr cloud_r (new PointCloud);

  // PREPARE SCENE AND MODEL
  auto orig_tree = unpackAndGetOctomap("fr_079");
  *orig_cloud = octreeToPointCloud(*orig_tree);
  printPointcloudInfo(*orig_cloud, "orig_cloud");

  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D (*orig_cloud, minPt, maxPt);

  auto cloud_min = Vector4f(-10, -10, -0.5, 1.0);
  auto cloud_max = Vector4f(7, 7, 3.0, 1.0);
  pcl::CropBox<Point> boxFilter;
  boxFilter.setMin(cloud_min);
  boxFilter.setMax(cloud_max);
  boxFilter.setInputCloud(orig_cloud);
  boxFilter.filter(*cropped_cloud);
  printPointcloudInfo(*cropped_cloud, "cropped_cloud");


  float x_common = 3;
  Vector4f center = (cloud_max + cloud_min) / 2;
  Vector4f cloud_l_max = {center(0) + x_common / 2, cloud_max(1), cloud_max(2), 1};
  Vector4f cloud_r_min = {center(0) - x_common / 2, cloud_min(1), cloud_min(2), 1};

  auto T = md::createTransformationMatrix(5.0, 0.0, 0.0, ToRadians(0), ToRadians(0), ToRadians(4));

  boxFilter.setMin(cloud_min);
  boxFilter.setMax(cloud_l_max);
  boxFilter.setInputCloud(cropped_cloud);
  boxFilter.filter(*cloud_l);

  boxFilter.setMin(cloud_r_min);
  boxFilter.setMax(cloud_max);
  boxFilter.setInputCloud(cropped_cloud);
  PointCloudPtr cloud_r_tmp (new PointCloud);
  boxFilter.filter(*cloud_r_tmp);
  pcl::transformPointCloud(*cloud_r_tmp, *cloud_r, T);

  CorrespondenceGroupingConfig config;
  config.use_hough_ = true;
  config.model_ss_ = 0.3;
  config.scene_ss_ = 0.1;
  config.rf_rad_ = 1.5;
  config.descr_rad_ = 1.5;
  config.cg_size_ = 0.5;
  config.cg_thresh_= 5.0;
  config.correspondences_thresh_ = 9;
  config.model_size_thresh_ = 50;
  config.cell_size_x_ = 2;
  config.cell_size_y_ = 2;

  auto start = high_resolution_clock::now();
  auto results = estimateTfsByCorrespondenceClustering(config, cloud_l, cloud_r);
  auto diff = duration_cast<milliseconds>(high_resolution_clock::now() - start);
  std::cout << "Time [ms]: " << diff.count() << "\n";
  results.PrintResults();

#if SHOW_PCL == 1
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor(255, 255, 255);
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  viewer.setCameraPosition(0.0, 0.0, 25.0, 0.0, 0.0, 0.0);

  // Visualize first octree as a point cloud --> BLUE
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_l, 0, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_l, color1, "tree1");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "tree1");

  // Visualize second octree as a point cloud --> RED
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud_r, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_r, color2, "tree2");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "tree2");

  // Visualize common part octree as a point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color4(cloud_model, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_model, color4, "tree_common");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "tree_common");
//  viewer.addCube(cell.min(0), cell.max(0), cell.min(1), cell.max(1), cloud_min(2), cloud_max(2), 0, 1, 0, "borders");
  pcl::PointXYZ pmin2, pmax2;
  pcl::getMinMax3D (*cloud_model, pmin2, pmax2);
  viewer.addCube(pmin2.x, pmax2.x, pmin2.y, pmax2.y, pmin2.z, pmax2.z, 1, 1, 0, "tree2_borders");

  for (size_t i = 0; i < cells_seq.size(); ++i) {
    auto cell = cells_seq[i];
    viewer.addCube(cell.min(0), cell.max(0), cell.min(1), cell.max(1), cloud_min(2), cloud_max(2), 0, 1, 0, "borders" + std::to_string(i));
  }


  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
#endif

}

/*TEST(IntegrateOctomaps, CorrespondenceGroupingPcl_Demo_PclVis_bac)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_l (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_common (new pcl::PointCloud<pcl::PointXYZ>);

#if LOAD_CLOUDS_FROM_FILES
  *cloud_l = *readPointCloudFromFile("fr_079_part1.pcd");
  *cloud_r = *readPointCloudFromFile("fr_079_part2.pcd");
  *cloud_common = *readPointCloudFromFile("fr_079_common.pcd");
#else
  // PREPARE SCENE AND MODEL
  auto orig_tree = unpackAndGetOctomap("fr_079");

  Vector3f tree_min = Vector3f{-10, -10, -0.5};
  Vector3f tree_max = Vector3f{7, 7, 3.0};
  auto tree = cutOctree(*orig_tree, tree_min, tree_max);

  float x_common = 3;
  Vector3f center = (tree_max + tree_min) / 2;
  Vector3f tree1_max = {center(0) + x_common / 2, tree_max(1), tree_max(2)};
  Vector3f tree2_min = {center(0) - x_common / 2, tree_min(1), tree_min(2)};

  auto T = md::createTransformationMatrix(5.0, 0.0, 0.0, ToRadians(0), ToRadians(0), ToRadians(0));

  OcTree tree_l = cutOctree(tree, tree_min, tree1_max);
  *cloud_l = octreeToPointCloud(tree_l);

  OcTree tree_r_tmp = cutOctree(tree, tree2_min, tree_max);
  OcTree tree_r = *(transformOctree(tree_r_tmp, T));
  auto cloud2i = octreeToPointCloud(tree_r_tmp);
  *cloud_r = octreeToPointCloud(tree_r);
//  pcl::transformPointCloud(cloud2i, cloud2, T);

  // Tree common
  Vector3f model_min = {center(0) - x_common / 2, tree_min(1) + 3, tree_min(2)};
  Vector3f model_max = {center(0) + x_common / 2, tree_max(1) - 3, tree_max(2)};
  OcTree tree_common_tmp = cutOctree(tree, model_min, model_max);
  auto T_common = md::createTransformationMatrix(20.0, 0.0, 0.0, ToRadians(0), ToRadians(0), ToRadians(0));
  OcTree tree_common = *(transformOctree(tree_common_tmp, T_common));
  *cloud_common = octreeToPointCloud(tree_common);

#endif

#if SAVE_CLOUDS_TO_FILES
  savePointCloudToFile("fr_079_part1.pcd", *cloud_l, true);
  savePointCloudToFile("fr_079_part2.pcd", *cloud_r, true);
  savePointCloudToFile("fr_079_common.pcd", *cloud_common, true);
#endif

#if SHOW_PCL == 1
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor(255, 255, 255);
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  viewer.setCameraPosition(0.0, 0.0, 25.0, 0.0, 0.0, 0.0);

  // Visualize first octree as a point cloud --> BLUE
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cloud_l, 0, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_l, color1, "tree1");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tree1");

  // Visualize second octree as a point cloud --> RED
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cloud_r, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_r, color2, "tree2");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tree2");

  // Visualize common part octree as a point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color4(cloud_common, 255, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cloud_common, color4, "tree_common");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tree_common");

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
#endif

#if CALC_CORRESPONDENCE
  //Algorithm params
  bool show_keypoints_ (true);
  bool show_correspondences_ (true);
  bool use_cloud_resolution_ (false);
  bool use_hough_ (true);
  float model_ss_ (0.3f);
  float scene_ss_ (0.1f);
  float rf_rad_ (1.5);
  float descr_rad_ (1.5);
  float cg_size_ (0.1f);
  float cg_thresh_ (5.0f);

  pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr model_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr scene_keypoints (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<NormalType>::Ptr model_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<NormalType>::Ptr scene_normals (new pcl::PointCloud<NormalType> ());
  pcl::PointCloud<DescriptorType>::Ptr model_descriptors (new pcl::PointCloud<DescriptorType> ());
  pcl::PointCloud<DescriptorType>::Ptr scene_descriptors (new pcl::PointCloud<DescriptorType> ());

  *model = *cloud_common;
  *scene = *cloud_l;

  //
  //  Set up resolution invariance
  //
  if (use_cloud_resolution_)
  {
    float resolution = static_cast<float> (computeCloudResolution (model));
    if (resolution != 0.0f)
    {
      model_ss_   *= resolution;
      scene_ss_   *= resolution;
      rf_rad_     *= resolution;
      descr_rad_  *= resolution;
      cg_size_    *= resolution;
    }

    std::cout << "Model resolution:       " << resolution << std::endl;
    std::cout << "Model sampling size:    " << model_ss_ << std::endl;
    std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
    std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
    std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
    std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
  }

  //
  //  Compute Normals
  //
  pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
  norm_est.setKSearch (10);
  norm_est.setInputCloud (model);
  norm_est.compute (*model_normals);

  norm_est.setInputCloud (scene);
  norm_est.compute (*scene_normals);
  //
  //  Downsample Clouds to Extract keypoints
  //

  pcl::UniformSampling<PointType> uniform_sampling;
  uniform_sampling.setInputCloud (model);
  uniform_sampling.setRadiusSearch (model_ss_);
  //uniform_sampling.filter (*model_keypoints);
  pcl::PointCloud<int> keypointIndices1;
  uniform_sampling.compute(keypointIndices1);
  pcl::copyPointCloud(*model, keypointIndices1.points, *model_keypoints);
  std::cout << "Model total points: " << model->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;

  uniform_sampling.setInputCloud (scene);
  uniform_sampling.setRadiusSearch (scene_ss_);
  //uniform_sampling.filter (*scene_keypoints);
  pcl::PointCloud<int> keypointIndices2;
  uniform_sampling.compute(keypointIndices2);
  pcl::copyPointCloud(*scene, keypointIndices2.points, *scene_keypoints);
  std::cout << "Scene total points: " << scene->size () << "; Selected Keypoints: " << scene_keypoints->size () << std::endl;

  //
  //  Compute Descriptor for keypoints
  //
  pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
  descr_est.setRadiusSearch (descr_rad_);

  descr_est.setInputCloud (model_keypoints);
  descr_est.setInputNormals (model_normals);
  descr_est.setSearchSurface (model);
  descr_est.compute (*model_descriptors);

  descr_est.setInputCloud (scene_keypoints);
  descr_est.setInputNormals (scene_normals);
  descr_est.setSearchSurface (scene);
  descr_est.compute (*scene_descriptors);

  //
  //  Find Model-Scene Correspondences with KdTree
  //
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model_descriptors);

  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
  for (size_t i = 0; i < scene_descriptors->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!std::isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
    }
  }
  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;

  //
  //  Actual Clustering
  //
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
  std::vector<pcl::Correspondences> clustered_corrs;

  //  Using Hough3D
  if (use_hough_)
  {
    //
    //  Compute (Keypoints) Reference Frames only for Hough
    //
    pcl::PointCloud<RFType>::Ptr model_rf (new pcl::PointCloud<RFType> ());
    pcl::PointCloud<RFType>::Ptr scene_rf (new pcl::PointCloud<RFType> ());

    pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
    rf_est.setFindHoles (true);
    rf_est.setRadiusSearch (rf_rad_);

    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    //  Clustering
    pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //clusterer.cluster (clustered_corrs);
    clusterer.recognize (rototranslations, clustered_corrs);
  }
  else // Using GeometricConsistency
  {
    pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints);
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  //
  //  Output results
  //
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
    std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size () << std::endl;

    // Print the rotation matrix and translation vector
    Eigen::Matrix3f rotation = rototranslations[i].block<3,3>(0, 0);
    Eigen::Vector3f translation = rototranslations[i].block<3,1>(0, 3);

    printf ("\n");
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
    printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
    printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
    printf ("\n");
    printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
  }

  //
  //  Visualization
  //
  pcl::visualization::PCLVisualizer viewer ("Correspondence Grouping");
  viewer.addPointCloud (scene, "scene_cloud");

  pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType> ());
  pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints (new pcl::PointCloud<PointType> ());

  if (show_correspondences_ || show_keypoints_)
  {
    //  We are translating the model so that it doesn't end in the middle of the scene representation
    pcl::transformPointCloud (*model, *off_scene_model, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));
    pcl::transformPointCloud (*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f (-1,0,0), Eigen::Quaternionf (1, 0, 0, 0));

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler (off_scene_model, 255, 255, 128);
    viewer.addPointCloud (off_scene_model, off_scene_model_color_handler, "off_scene_model");
  }

  if (show_keypoints_)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler (scene_keypoints, 0, 0, 255);
    viewer.addPointCloud (scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler (off_scene_model_keypoints, 0, 0, 255);
    viewer.addPointCloud (off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
  }

  for (size_t i = 0; i < rototranslations.size (); ++i)
  {
    pcl::PointCloud<PointType>::Ptr rotated_model (new pcl::PointCloud<PointType> ());
    pcl::transformPointCloud (*model, *rotated_model, rototranslations[i]);

    std::stringstream ss_cloud;
    ss_cloud << "instance" << i;

    pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler (rotated_model, 255, 0, 0);
    viewer.addPointCloud (rotated_model, rotated_model_color_handler, ss_cloud.str ());

    if (show_correspondences_)
    {
      for (size_t j = 0; j < clustered_corrs[i].size (); ++j)
      {
        std::stringstream ss_line;
        ss_line << "correspondence_line" << i << "_" << j;
        PointType& model_point = off_scene_model_keypoints->at (clustered_corrs[i][j].index_query);
        PointType& scene_point = scene_keypoints->at (clustered_corrs[i][j].index_match);

        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
        viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str ());
      }
    }
  }

  while (!viewer.wasStopped ())
  {
    viewer.spinOnce ();
  }

#endif
}*/


TEST(IntegrateOctomaps, CutAndSavePointcloudsPcl_Demo_PclVis)
{
  auto orig_tree = unpackAndGetOctomap("fr_079");

  Vector3f tree_min = Vector3f{-10, -10, -0.5};
  Vector3f tree_max = Vector3f{7, 7, 3.0};
  auto tree = cutOctree(*orig_tree, tree_min, tree_max);

  float x_common = 4;
  Vector3f center = (tree_max + tree_min) / 2;
  Vector3f tree1_max = {center(0) + x_common / 2, tree_max(1), tree_max(2)};
  Vector3f tree2_min = {center(0) - x_common / 2, tree_min(1), tree_min(2)};

  auto T = md::createTransformationMatrix(5.0, 0.1, 0.1, ToRadians(1), ToRadians(1), ToRadians(5));

  OcTree tree_l = cutOctree(tree, tree_min, tree1_max);
  auto cloud_l = octreeToPointCloud(tree_l);

  OcTree tree_r_tmp = cutOctree(tree, tree2_min, tree_max);
  OcTree tree_r = *(transformOctree(tree_r_tmp, T));
  auto cloud2i = octreeToPointCloud(tree_r_tmp);
  PointCloud cloud2 = octreeToPointCloud(tree_r);
//  pcl::transformPointCloud(cloud2i, cloud2, T);

  // Tree common
  OcTree tree_common_tmp = cutOctree(tree, tree2_min, tree1_max);
  auto T_common = md::createTransformationMatrix(25.0, 0.5, 0.1, ToRadians(1), ToRadians(1), ToRadians(5));
  OcTree tree_common = *(transformOctree(tree_common_tmp, T_common));
  auto cloud_common = octreeToPointCloud(tree_common);

#if SAVE_CLOUDS_TO_FILES
  savePointCloudToFile("fr_079_part1.pcd", cloud_l, true);
  savePointCloudToFile("fr_079_part2.pcd", cloud2, true);
  savePointCloudToFile("fr_079_common.pcd", cloud_common, true);
#endif

//  pcl::PointXYZ margin = {0.5, 0.5, 0.5};
//  OctreeIntegrationConf conf {100, 0.8, 0.05, margin, 0.001, 0.04 };
//  Eigen::Matrix4f T_fin;
//  {
//    auto start = std::chrono::high_resolution_clock::now();
//    T_fin = estimateTransBetweenPointclouds(cloud_l, cloud2, conf);
//    auto diff = std::chrono::high_resolution_clock::now() - start;
//
//    printTransformation(T, "Applied transformation");
//    printTransformation(T_fin, "Estimated transformation");
//    std::cout << "\nError: " << transoformationsError(T, T_fin);
//    std::cout << "\nTime: " << ToMilliseconds(diff) << " ms \n";
//  }
//
//  {
//    auto start = std::chrono::high_resolution_clock::now();
//    T_fin = estimateTransBetweenOctomapsPcl(tree_l, tree_r, conf);
//    auto diff = std::chrono::high_resolution_clock::now() - start;
//
//    printTransformation(T, "Applied transformation");
//    printTransformation(T_fin, "Estimated transformation");
//    std::cout << "\nError: " << transoformationsError(T, T_fin);
//    std::cout << "\nTime: " << ToMilliseconds(diff) << " ms \n";
//  }

#if SHOW_PCL == 1
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor(255, 255, 255);
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  viewer.setCameraPosition(0.0, 0.0, 25.0, 0.0, 0.0, 0.0);

  // Visualize first octree as a point cloud --> BLUE
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld1 (new pcl::PointCloud<pcl::PointXYZ>);
  *cld1 = octreeToPointCloud(tree_l);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cld1, 0, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ>(cld1, color1, "tree1");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tree1");
  Vector3f pmin1, pmax1;
  getMinMaxOctree(tree_l, pmin1, pmax1);
  viewer.addCube(pmin1(0), pmax1(0), pmin1(1), pmax1(1), pmin1(2), pmax1(2), 0, 0, 1, "tree1_borders");

  // Visualize second octree as a point cloud --> RED
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld2 (new pcl::PointCloud<pcl::PointXYZ>);
  *cld2 = octreeToPointCloud(tree_r);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cld2, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cld2, color2, "tree2");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tree2");
  Vector3f pmin2, pmax2;
  getMinMaxOctree(tree_r, pmin2, pmax2);
  viewer.addCube(pmin2(0), pmax2(0), pmin2(1), pmax2(1), pmin2(2), pmax2(2), 1, 0, 0, "tree2_borders");

//  // Transform second tree with estimated transformation and visualize it --> GREEN
//  auto transf_tree = transformOctree(tree_l, T_fin);
////  printOcTreeInfo(*transf_tree, "trans_tree");
//  pcl::PointCloud<pcl::PointXYZ>::Ptr cld3 (new pcl::PointCloud<pcl::PointXYZ>);
//  pcl::transformPointCloud(cloud_l, *cld3, T_fin);
//  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cld2, 0, 255, 0);
//  viewer.addPointCloud<pcl::PointXYZ> (cld3, color3, "tree_transformed");
//  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tree_transformed");
//  Vector3f pmin3, pmax3;
//  getMinMaxOctree(*transf_tree, pmin3, pmax3);
//  viewer.addCube(pmin3(0), pmax3(0), pmin3(1), pmax3(1), pmin3(2), pmax3(2), 0, 1, 0, "tree3_borders");
//
//  viewer.addCube(tree_min(0), tree_max(0),
//                 tree_min(1), tree_max(1), tree_min(2), tree_max(2), 1, 1, 0, "tree_borders");

  // Visualize common part octree as a point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld4 (new pcl::PointCloud<pcl::PointXYZ>);
  *cld4 = octreeToPointCloud(tree_common);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color4(cld4, 255, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cld4, color4, "tree_common");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tree_common");
  Vector3f pmin4, pmax4;
  getMinMaxOctree(tree_common, pmin2, pmax2);
  viewer.addCube(pmin4(0), pmax4(0), pmin4(1), pmax4(1), pmin4(2), pmax4(2), 1, 0, 0, "tree_common_borders");


  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
#endif
}
