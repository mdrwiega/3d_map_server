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

using namespace Eigen;
using namespace octomap_tools;
using namespace octomap;
using namespace md;

#define SHOW_IMAGES 0
#define SHOW_PCL 1
#define RUN_OCTOVIS 0

TEST(IntegrateOctomaps, Correspondence_groupingPcl_Demo_PclVis)
{
  auto orig_tree = unpackAndGetOctomap("fr_079");

  Vector3f tree_min = Vector3f{-10, -10, -0.5};
  Vector3f tree_max = Vector3f{7, 7, 3.0};
  auto tree = cutOctree(*orig_tree, tree_min, tree_max);

  Vector3f center = (tree_max + tree_min) / 2;
  Vector3f tree1_max = {center(0) + 2.f, tree_max(1), tree_max(2)};
  Vector3f tree2_min = {center(0) - 2.f, tree_min(1), tree_min(2)};

  auto T = md::createTransformationMatrix(0.1, 0.1, 0.1, ToRadians(1), ToRadians(1), ToRadians(5));

  OcTree tree_l = cutOctree(tree, tree_min, tree1_max);
  auto cloud_l = octreeToPointCloud(tree_l);

  OcTree tree_r_tmp = cutOctree(tree, tree2_min, tree_max);
  OcTree tree_r = *(transformOctree(tree_r_tmp, T));
  auto cloud2i = octreeToPointCloud(tree_r_tmp);
  PointCloud cloud2 = octreeToPointCloud(tree_r);
//  pcl::transformPointCloud(cloud2i, cloud2, T);

  pcl::PointXYZ margin = {0.5, 0.5, 0.5};
  OctreeIntegrationConf conf {100, 0.8, 0.05, margin, 0.001, 0.04 };
  Eigen::Matrix4f T_fin;
  {
    auto start = std::chrono::high_resolution_clock::now();
    T_fin = estimateTransBetweenPointclouds(cloud_l, cloud2, conf);
    auto diff = std::chrono::high_resolution_clock::now() - start;

    printTransformation(T, "Applied transformation");
    printTransformation(T_fin, "Estimated transformation");
    std::cout << "\nError: " << transoformationsError(T, T_fin);
    std::cout << "\nTime: " << ToMilliseconds(diff) << " ms \n";
  }

  {
    auto start = std::chrono::high_resolution_clock::now();
    T_fin = estimateTransBetweenOctomapsPcl(tree_l, tree_r, conf);
    auto diff = std::chrono::high_resolution_clock::now() - start;

    printTransformation(T, "Applied transformation");
    printTransformation(T_fin, "Estimated transformation");
    std::cout << "\nError: " << transoformationsError(T, T_fin);
    std::cout << "\nTime: " << ToMilliseconds(diff) << " ms \n";
  }

#if SHOW_PCL == 1
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor(255, 255, 255);
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  viewer.setCameraPosition(0.0, 0.0, 25.0, 0.0, 0.0, 0.0);

  // Visualize first octree as a point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld1 (new pcl::PointCloud<pcl::PointXYZ>);
  *cld1 = octreeToPointCloud(tree_l);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cld1, 0, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ>(cld1, color1, "tree1");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tree1");
  Vector3f pmin1, pmax1;
  getMinMaxOctree(tree_l, pmin1, pmax1);
  viewer.addCube(pmin1(0), pmax1(0), pmin1(1), pmax1(1), pmin1(2), pmax1(2), 0, 0, 1, "tree1_borders");

  // Visualize second octree as a point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld2 (new pcl::PointCloud<pcl::PointXYZ>);
  *cld2 = octreeToPointCloud(tree_r);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cld2, 255, 0, 0);
  viewer.addPointCloud<pcl::PointXYZ>(cld2, color2, "tree2");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tree2");
  Vector3f pmin2, pmax2;
  getMinMaxOctree(tree_r, pmin2, pmax2);
  viewer.addCube(pmin2(0), pmax2(0), pmin2(1), pmax2(1), pmin2(2), pmax2(2), 1, 0, 0, "tree2_borders");

  // Transform second tree with estimated transformation and visualize it
  auto transf_tree = transformOctree(tree_l, T_fin);
//  printOcTreeInfo(*transf_tree, "trans_tree");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld3 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(cloud_l, *cld3, T_fin);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cld2, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (cld3, color3, "tree_transformed");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tree_transformed");
  Vector3f pmin3, pmax3;
  getMinMaxOctree(*transf_tree, pmin3, pmax3);
  viewer.addCube(pmin3(0), pmax3(0), pmin3(1), pmax3(1), pmin3(2), pmax3(2), 0, 1, 0, "tree3_borders");

  viewer.addCube(tree_min(0), tree_max(0),
                 tree_min(1), tree_max(1), tree_min(2), tree_max(2), 1, 1, 0, "tree_borders");

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
#endif
}
