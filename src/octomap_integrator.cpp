#include "octomap_integrator.h"

#include <thread>

#include "octomap_merger.h"
#include "octree_transformations.h"
#include "octree_icp.h"
#include "md_utils/math/transformations.h"
#include "utils/types_conversions.h"
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>

#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>
//#include <pcl/filters/uniform_sampling.h>

#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/registration/ia_ransac.h>

typedef pcl::ReferenceFrame RFType;
using NormalType = pcl::Normal;
using SurfaceNormals = pcl::PointCloud<pcl::Normal>;
using DescriptorType = pcl::FPFHSignature33; // SHOT352;
using Descriptors = pcl::PointCloud<DescriptorType>;

#define SHOW_PCL 0

namespace octomap_tools {

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ed4b629... Next part of refactoring
std::vector<Eigen::Vector2f> generateSpiralCoordinatesSequence(int size_x, int size_y) {
  std::vector<Eigen::Vector2f> seq;
  int m = size_x;
  int n = size_y;
  int i, k = 0, l = 0;

  /*  k - starting row index
      m - ending row index
      l - starting column index
      n - ending column index
      i - iterator
  */

  while (k < m && l < n) {
      /* Print the first row from the remaining rows */
      for (i = l; i < n; ++i) {
        seq.push_back(Eigen::Vector2f(k, i));
      }
      k++;
<<<<<<< HEAD

      /* Print the last column from the remaining columns */
      for (i = k; i < m; ++i) {
          seq.push_back(Eigen::Vector2f(i, n-1));
      }
      n--;

      /* Print the last row from the remaining rows */
      if ( k < m) {
          for (i = n-1; i >= l; --i) {
            seq.push_back(Eigen::Vector2f(m-1, i));
          }
          m--;
      }

      /* Print the first column from the remaining columns */
      if (l < n) {
          for (i = m-1; i >= k; --i) {
            seq.push_back(Eigen::Vector2f(i, l));
          }
          l++;
      }
  }
  return seq;
}

std::vector<Rectangle> generateBlocksInSpiralOrder(Eigen::Vector2f& min, Eigen::Vector2f& max, Eigen::Vector2f& step_xy)
{
  std::vector<Rectangle> cells;
  int size_x = ceil((max(0) - min(0)) / step_xy(0)) ;
  int size_y = ceil((max(1) - min(1)) / step_xy(1)) ;
  std::vector<Eigen::Vector2f> seq = generateSpiralCoordinatesSequence(size_x, size_y);

  for (auto& i : seq) {
    Rectangle rect;
    rect.min(0) = min(0) + i(0) * step_xy(0);
    rect.min(1) = min(1) + i(1) * step_xy(1);
    rect.max(0) = min(0) + (i(0) + 1) * step_xy(0);
    rect.max(1) = min(1) + (i(1) + 1) * step_xy(1);

    if (rect.max(0) > max(0)) rect.max(0) = max(0);
    if (rect.max(1) > max(1)) rect.max(1) = max(1);

    cells.push_back(rect);
  }

  return cells;
}
=======
void calc(FeatureCloudPtr& scene,
          FeatureCloudPtr& model,
          const CorrespondenceGroupingConfig& config,
          CGResultsSet& results) {
  //  Find Model-Scene Correspondences with KdTree
  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());

  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud (model->getDescriptors());
>>>>>>> bd57a90... Next part of refactoring about feature detection method
=======

      /* Print the last column from the remaining columns */
      for (i = k; i < m; ++i) {
          seq.push_back(Eigen::Vector2f(i, n-1));
      }
      n--;

      /* Print the last row from the remaining rows */
      if ( k < m) {
          for (i = n-1; i >= l; --i) {
            seq.push_back(Eigen::Vector2f(m-1, i));
          }
          m--;
      }

      /* Print the first column from the remaining columns */
      if (l < n) {
          for (i = m-1; i >= k; --i) {
            seq.push_back(Eigen::Vector2f(i, l));
          }
          l++;
      }
  }
  return seq;
}

std::vector<Rectangle> generateBlocksInSpiralOrder(Eigen::Vector2f& min, Eigen::Vector2f& max, Eigen::Vector2f& step_xy)
{
  std::vector<Rectangle> cells;
  int size_x = ceil((max(0) - min(0)) / step_xy(0)) ;
  int size_y = ceil((max(1) - min(1)) / step_xy(1)) ;
  std::vector<Eigen::Vector2f> seq = generateSpiralCoordinatesSequence(size_x, size_y);

  for (auto& i : seq) {
    Rectangle rect;
    rect.min(0) = min(0) + i(0) * step_xy(0);
    rect.min(1) = min(1) + i(1) * step_xy(1);
    rect.max(0) = min(0) + (i(0) + 1) * step_xy(0);
    rect.max(1) = min(1) + (i(1) + 1) * step_xy(1);

    if (rect.max(0) > max(0)) rect.max(0) = max(0);
    if (rect.max(1) > max(1)) rect.max(1) = max(1);

    cells.push_back(rect);
  }

  return cells;
}
>>>>>>> ed4b629... Next part of refactoring

void HoughClustering(
    const CorrespondenceGroupingConfig& config,
    const pcl::CorrespondencesPtr& model_scene_corrs,
    FeatureCloudPtr& model,
    FeatureCloudPtr& scene,
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,
    std::vector<pcl::Correspondences>& clustered_corrs) {
  //  Compute (Keypoints) Reference Frames only for Hough
  pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
  pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());
  pcl::BOARDLocalReferenceFrameEstimation<Point, NormalType, RFType> rf_est;
  rf_est.setFindHoles(true);
  rf_est.setRadiusSearch(config.rf_rad_);
  rf_est.setInputCloud(model->getKeypoints());
  rf_est.setInputNormals(model->getSurfaceNormals());
  rf_est.setSearchSurface(model->getPointCloud());
  rf_est.compute(*model_rf);
  rf_est.setInputCloud(scene->getKeypoints());
  rf_est.setInputNormals(scene->getSurfaceNormals());
  rf_est.setSearchSurface(scene->getPointCloud());
  rf_est.compute(*scene_rf);
  //  Clustering
  pcl::Hough3DGrouping<Point, Point, RFType, RFType> clusterer;
  clusterer.setHoughBinSize(config.cg_size_);
  clusterer.setHoughThreshold(config.cg_thresh_);
  clusterer.setUseInterpolation(true);
  clusterer.setUseDistanceWeight(false);
  clusterer.setInputCloud(model->getKeypoints());
  clusterer.setInputRf(model_rf);
  clusterer.setSceneCloud(scene->getKeypoints());
  clusterer.setSceneRf(scene_rf);
  clusterer.setModelSceneCorrespondences(model_scene_corrs);
  //clusterer.cluster (clustered_corrs);
  clusterer.recognize(rototranslations, clustered_corrs);
}
<<<<<<< HEAD

void GeometryConsistencyGrouping(
    const CorrespondenceGroupingConfig& config,
    const pcl::CorrespondencesPtr& model_scene_corrs,
    FeatureCloudPtr& model,
    FeatureCloudPtr& scene,
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,
    std::vector<pcl::Correspondences>& clustered_corrs) {
  pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
  gc_clusterer.setGCSize(config.cg_size_);
  gc_clusterer.setGCThreshold(config.cg_thresh_);
  gc_clusterer.setInputCloud(model->getKeypoints());
  gc_clusterer.setSceneCloud(scene->getKeypoints());
  gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);
  //gc_clusterer.cluster (clustered_corrs);
  gc_clusterer.recognize(rototranslations, clustered_corrs);
}

=======

void GeometryConsistencyGrouping(
    const CorrespondenceGroupingConfig& config,
    const pcl::CorrespondencesPtr& model_scene_corrs,
    FeatureCloudPtr& model,
    FeatureCloudPtr& scene,
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,
    std::vector<pcl::Correspondences>& clustered_corrs) {
  pcl::GeometricConsistencyGrouping<Point, Point> gc_clusterer;
  gc_clusterer.setGCSize(config.cg_size_);
  gc_clusterer.setGCThreshold(config.cg_thresh_);
  gc_clusterer.setInputCloud(model->getKeypoints());
  gc_clusterer.setSceneCloud(scene->getKeypoints());
  gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);
  //gc_clusterer.cluster (clustered_corrs);
  gc_clusterer.recognize(rototranslations, clustered_corrs);
}

>>>>>>> ed4b629... Next part of refactoring
void FindCorrespondencesWithKdTree(const pcl::CorrespondencesPtr& model_scene_corrs,
                         FeatureCloudPtr& model, FeatureCloudPtr& scene) {
  pcl::KdTreeFLANN<DescriptorType> match_search;
  match_search.setInputCloud(model->getDescriptors());
  //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ed4b629... Next part of refactoring
  for (size_t i = 0; i < scene->getDescriptors()->size(); ++i) {
    auto& scene_descriptor = scene->getDescriptors()->at(i);
    std::vector<int> neigh_indices(1);
    std::vector<float> neigh_sqr_dists(1);
    if (!std::isfinite(scene_descriptor.descriptor[0])) {
      //skipping NaNs
<<<<<<< HEAD
      continue;
    }
    int found_neighs = match_search.nearestKSearch(scene_descriptor, 1,
                                                   neigh_indices,
                                                   neigh_sqr_dists);
    if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)  //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
      pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i),
                               neigh_sqr_dists[0]);
      model_scene_corrs->push_back(corr);
=======
  for (size_t i = 0; i < scene->getDescriptors()->size (); ++i)
  {
    std::vector<int> neigh_indices (1);
    std::vector<float> neigh_sqr_dists (1);
    if (!std::isfinite (scene->getDescriptors()->at (i).descriptor[0])) //skipping NaNs
    {
      continue;
    }
    int found_neighs = match_search.nearestKSearch (scene->getDescriptors()->at (i), 1, neigh_indices, neigh_sqr_dists);
    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
    {
      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
      model_scene_corrs->push_back (corr);
>>>>>>> bd57a90... Next part of refactoring about feature detection method
=======
      continue;
    }
    int found_neighs = match_search.nearestKSearch(scene_descriptor, 1,
                                                   neigh_indices,
                                                   neigh_sqr_dists);
    if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f)  //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        {
      pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i),
                               neigh_sqr_dists[0]);
      model_scene_corrs->push_back(corr);
>>>>>>> ed4b629... Next part of refactoring
    }
  }
}

void calc(FeatureCloudPtr& scene, FeatureCloudPtr& model,
          const CorrespondenceGroupingConfig& config, CGResultsSet& results) {

  pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences());
  FindCorrespondencesWithKdTree(model_scene_corrs, model, scene);

  std::cout << "Correspondences found: " << model_scene_corrs->size () << std::endl;
  if (model_scene_corrs->size() < config.correspondences_thresh_) {
    std::cout << "Correspondences below threshold. Exit." << std::endl;
    return;
  }
  //  Actual Clustering
<<<<<<< HEAD
<<<<<<< HEAD
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
  std::vector<pcl::Correspondences> clustered_corrs;

  if (config.use_hough_) {
    HoughClustering(config, model_scene_corrs, model, scene, transformations, clustered_corrs);
  } else {
    GeometryConsistencyGrouping(config, model_scene_corrs, model, scene,
                                transformations, clustered_corrs);
  }

  std::cout << "Model instances found: " << transformations.size () << std::endl;
  if (transformations.size() == 0) {
=======
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
=======
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > transformations;
>>>>>>> ed4b629... Next part of refactoring
  std::vector<pcl::Correspondences> clustered_corrs;

  if (config.use_hough_) {
    HoughClustering(config, model_scene_corrs, model, scene, transformations, clustered_corrs);
  } else {
    GeometryConsistencyGrouping(config, model_scene_corrs, model, scene,
                                transformations, clustered_corrs);
  }

<<<<<<< HEAD
    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
  }

  //  Output results
  std::cout << "Model instances found: " << rototranslations.size () << std::endl;
  if (rototranslations.size() == 0) {
>>>>>>> bd57a90... Next part of refactoring about feature detection method
=======
  std::cout << "Model instances found: " << transformations.size () << std::endl;
  if (transformations.size() == 0) {
>>>>>>> ed4b629... Next part of refactoring
    std::cout << "No model instances. Exit" << std::endl;
    return;
  }

  Point pmin, pmax;
  pcl::getMinMax3D(*model->getPointCloud(), pmin, pmax);
  // Copy results to data structure
<<<<<<< HEAD
<<<<<<< HEAD
  for (size_t i = 0; i < transformations.size (); ++i) {
=======
  for (size_t i = 0; i < rototranslations.size (); ++i) {
>>>>>>> bd57a90... Next part of refactoring about feature detection method
=======
  for (size_t i = 0; i < transformations.size (); ++i) {
>>>>>>> ed4b629... Next part of refactoring
    CGResultEntry result;
    result.model_max = pmax;
    result.model_min = pmin;
    result.transformation = transformations[i];
    result.correspondences = clustered_corrs[i].size();
    results.AppendResultEntry(result);
  }

<<<<<<< HEAD
<<<<<<< HEAD
  visualize(config, transformations, clustered_corrs, scene, model);
=======
  visualize(config, rototranslations, clustered_corrs, scene, model);
>>>>>>> bd57a90... Next part of refactoring about feature detection method
=======
  visualize(config, transformations, clustered_corrs, scene, model);
>>>>>>> ed4b629... Next part of refactoring
}

void visualize(
    const CorrespondenceGroupingConfig& config,
    const std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >& rototranslations,
    const std::vector<pcl::Correspondences>& clustered_corrs,
    FeatureCloudPtr& scene, FeatureCloudPtr& model) {
<<<<<<< HEAD
<<<<<<< HEAD

  if (config.show_visualization_) {
    pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
    viewer.addPointCloud(scene->getPointCloud(), "scene_cloud");
    PointCloud::Ptr off_scene_model(new PointCloud());
    PointCloud::Ptr off_scene_model_keypoints(
        new PointCloud());
=======
  //
  //  Visualization
  //
  if (config.show_visualization_) {
    pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
    viewer.addPointCloud(scene->getPointCloud(), "scene_cloud");
    pcl::PointCloud<Point>::Ptr off_scene_model(new pcl::PointCloud<Point>());
    pcl::PointCloud<Point>::Ptr off_scene_model_keypoints(
        new pcl::PointCloud<Point>());
>>>>>>> bd57a90... Next part of refactoring about feature detection method
=======

  if (config.show_visualization_) {
    pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
    viewer.addPointCloud(scene->getPointCloud(), "scene_cloud");
    PointCloud::Ptr off_scene_model(new PointCloud());
    PointCloud::Ptr off_scene_model_keypoints(
        new PointCloud());
>>>>>>> ed4b629... Next part of refactoring
    if (config.show_correspondences_ || config.show_keypoints_) {
      //  We are translating the model so that it doesn't end in the middle of the scene representation
      pcl::transformPointCloud(*model->getPointCloud(), *off_scene_model,
                               Eigen::Vector3f(-1, 0, 0),
                               Eigen::Quaternionf(1, 0, 0, 0));
      pcl::transformPointCloud(*model->getKeypoints(),
                               *off_scene_model_keypoints,
                               Eigen::Vector3f(-1, 0, 0),
                               Eigen::Quaternionf(1, 0, 0, 0));
      pcl::visualization::PointCloudColorHandlerCustom<Point> off_scene_model_color_handler(
          off_scene_model, 255, 255, 128);
      viewer.addPointCloud(off_scene_model, off_scene_model_color_handler,
                           "off_scene_model");
    }
    if (config.show_keypoints_) {
      pcl::visualization::PointCloudColorHandlerCustom<Point> scene_keypoints_color_handler(
          scene->getKeypoints(), 0, 0, 255);
      viewer.addPointCloud(scene->getKeypoints(), scene_keypoints_color_handler,
                           "scene_keypoints");
      viewer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
      pcl::visualization::PointCloudColorHandlerCustom<Point> off_scene_model_keypoints_color_handler(
          off_scene_model_keypoints, 0, 0, 255);
      viewer.addPointCloud(off_scene_model_keypoints,
                           off_scene_model_keypoints_color_handler,
                           "off_scene_model_keypoints");
      viewer.setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
          "off_scene_model_keypoints");
    }
    for (size_t i = 0; i < rototranslations.size(); ++i) {
<<<<<<< HEAD
<<<<<<< HEAD
      PointCloud::Ptr rotated_model(new PointCloud());
=======
      pcl::PointCloud<Point>::Ptr rotated_model(new pcl::PointCloud<Point>());
>>>>>>> bd57a90... Next part of refactoring about feature detection method
=======
      PointCloud::Ptr rotated_model(new PointCloud());
>>>>>>> ed4b629... Next part of refactoring
      pcl::transformPointCloud(*model->getPointCloud(), *rotated_model,
                               rototranslations[i]);
      std::stringstream ss_cloud;
      ss_cloud << "instance" << i;
      pcl::visualization::PointCloudColorHandlerCustom<Point> rotated_model_color_handler(
          rotated_model, 255, 0, 0);
      viewer.addPointCloud(rotated_model, rotated_model_color_handler,
                           ss_cloud.str());
      if (config.show_correspondences_) {
        for (size_t j = 0; j < clustered_corrs[i].size(); ++j) {
          std::stringstream ss_line;
          ss_line << "correspondence_line" << i << "_" << j;
          Point& model_point = off_scene_model_keypoints->at(
              clustered_corrs[i][j].index_query);
          Point& scene_point = scene->getKeypoints()->at(
              clustered_corrs[i][j].index_match);
          //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
          viewer.addLine<Point, Point>(model_point, scene_point, 0, 255, 0,
                                       ss_line.str());
        }
      }
    }
    while (!viewer.wasStopped()) {
      viewer.spinOnce();
    }
  }
}


=======
>>>>>>> 5841953... Refactoring
OcTreePtr integrateOctomaps(const OcTree& tree1, const OcTree& tree2,
                            const OctreeIntegrationConf& conf,
                            const Eigen::Matrix4f& T_init,
                            Eigen::Matrix4f& T_fin,
                            float& error)
{
  T_fin = estimateTransBetweenOctomaps(tree1, tree2, conf);
  LOG_DBG() << "ICP has been finished\nEstimated rotation: "
      << T_fin.block<3,3>(0,0).eulerAngles(0, 1, 2).transpose()
      << "\nEstimated translation: "
      << T_fin.block<3,1>(0,3).transpose() << "\n";

  auto trans_src_tree = transformOctree(tree1, T_fin);
  LOG_DBG() << "Octree has been transformed\n";

  auto merged_tree = sumOctrees(*trans_src_tree, tree2);
  LOG_DBG() << "Octrees have been merged\n";
  return merged_tree;
}

Eigen::Matrix4f estimateTransBetweenOctomaps(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf)
{
  OcTree src_tree_f(1);
  OcTree dst_tree_f(1);

  extractIntersectingOctrees(
      tree1, tree2, conf.intersec_margin, src_tree_f, dst_tree_f);

  return icp(src_tree_f, dst_tree_f, conf.max_iter, conf.fitness_eps);
}

OcTreePtr integrateOctomapsPcl(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf,
    const Eigen::Matrix4f& T_init, Eigen::Matrix4f& T_fin,
    float& error)
{
  PointCloud cloud_src = octreeToPointCloud(tree1);
  PointCloud cloud_dst = octreeToPointCloud(tree2);

  T_fin = estimateTransBetweenPointclouds(cloud_src, cloud_dst, conf);
  LOG_DBG() << "ICP has been finished\nEstimated rotation: "
      << T_fin.block<3,3>(0,0).eulerAngles(0, 1, 2).transpose()
      << "\nEstimated translation: "
      << T_fin.block<3,1>(0,3).transpose() << "\n";

  auto trans_src_tree = transformOctree(tree1, T_fin);
  LOG_DBG() << "Octree has been transformed\n";

  auto merged_tree = sumOctrees(*trans_src_tree, tree2);
  LOG_DBG() << "Octrees have been merged\n";
  return merged_tree;
}

Eigen::Matrix4f estimateTransBetweenOctomapsPcl(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf)
{
  auto cloud1 = octreeToPointCloud(tree1);
  auto cloud2 = octreeToPointCloud(tree2);
  return estimateTransBetweenPointclouds(cloud1, cloud2, conf);
}

Eigen::Matrix4f estimateTransBetweenPointclouds(
    const PointCloud& cloud1, const PointCloud& cloud2,
    const OctreeIntegrationConf& params)
{
  PointCloud::Ptr source(new PointCloud);
  PointCloud::Ptr target(new PointCloud);

  extractIntersectingAndDownsamplePointClouds(
      cloud1, cloud2, params.voxel_size, params.intersec_margin, *source, *target);

#if SHOW_PCL == 1
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters();
  viewer.setCameraPosition(0.0, 0.0, 25.0, 0.0, 0.0, 0.0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(source, 0, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ> (source, single_color, "source");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(target, 255, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ> (target, single_color2, "target");

  Point min1, max1, min2, max2;
  pcl::getMinMax3D(*source, min1, max1);
  pcl::getMinMax3D(*target, min2, max2);

  viewer.addCube(min1.x, max1.x, min1.y, max1.y, min1.z, max1.z, 0, 0, 1, "source");
  viewer.addCube(min2.x, max2.x, min2.y, max2.y, min2.z, max2.z, 1, 0, 1, "target");
#endif

  pcl::IterativeClosestPoint <Point, Point> icp;
  icp.setMaxCorrespondenceDistance(params.max_nn_dist);
  icp.setMaximumIterations(params.max_iter);
  icp.setTransformationEpsilon(params.transf_eps);
  icp.setEuclideanFitnessEpsilon (params.fitness_eps);

  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.align(*source);
  std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
  std::cout << "ICP transformation " << " : cloud_icp -> cloud_in" << std::endl;

#if SHOW_PCL == 1
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(source, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (source, single_color3, "src");
  pcl::getMinMax3D(*source, min1, max1);
  viewer.addCube(min1.x, max1.x, min1.y, max1.y, min1.z, max1.z, 0, 1, 0, "source2");

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
#endif

  return icp.getFinalTransformation();
}

}
