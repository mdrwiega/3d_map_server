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

TEST(IntegrateOctomaps, EstimationOnlyWithPcl_Demo_PclVis_PwrLab)
{
  auto orig_tree = unpackAndGetOctomap("2019.02.04.13.16.18_pwr_lab509");

  Vector3f tree_min = Vector3f{-10, -10, 0.1};
  Vector3f tree_max = Vector3f{11, 11, 0.7};
  auto tree = cutOctree(*orig_tree, tree_min, tree_max);
  printOcTreeInfo(tree, "Tree after cut");

  Vector3f center = (tree_max + tree_min) / 2;
  Vector3f tree1_max = {center(0) + .7f, tree_max(1), tree_max(2)};
  Vector3f tree2_min = {center(0) - .7f, tree_min(1), tree_min(2)};

  auto T = md::createTransformationMatrix(0.1, 0.1, 0.02, ToRadians(0.5), ToRadians(0.5), ToRadians(2));

  OcTree tree_l = cutOctree(tree, tree_min, tree1_max);
  auto cloud_l = octreeToPointCloud(tree_l);

  OcTree tree_r_tmp = cutOctree(tree, tree2_min, tree_max);
  OcTree tree_r = *(transformOctree(tree_r_tmp, T));
  auto cloud2i = octreeToPointCloud(tree_r_tmp);
  PointCloud cloud2 = octreeToPointCloud(tree_r);
//  pcl::transformPointCloud(cloud2i, cloud2, T);

  pcl::PointXYZ margin = {1.0, 1.0, 0.5};
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

TEST(IntegrateOctomaps, EstimationOnlyWithPcl_Demo_PclVis)
{
  auto orig_tree = unpackAndGetOctomap("fr_079");

<<<<<<< HEAD
  Vector3f tree_min = Vector3f{-6, -5, 0.1};
  Vector3f tree_max = Vector3f{6, 5, 0.6};
=======
  Vector3f tree_min = Vector3f{-10, -10, -0.5};
  Vector3f tree_max = Vector3f{7, 7, 3.0};
>>>>>>> 77a5caf... Package can be compiled now.
  auto tree = cutOctree(*orig_tree, tree_min, tree_max);

  Vector3f center = (tree_max + tree_min) / 2;
  Vector3f tree1_max = {center(0) + 1.5f, tree_max(1), tree_max(2)};
  Vector3f tree2_min = {center(0) - 1.5f, tree_min(1), tree_min(2)};

<<<<<<< HEAD
  auto T = md::createTransformationMatrix(0.3, 0.15, 0, 0, 0, ToRadians(0));
=======
  auto T = md::createTransformationMatrix(0.1, 0.1, 0.1, ToRadians(1), ToRadians(1), ToRadians(5));
>>>>>>> 77a5caf... Package can be compiled now.

  OcTree tree_l = cutOctree(tree, tree_min, tree1_max);
  auto cloud_l = octreeToPointCloud(tree_l);

  OcTree tree_r_tmp = cutOctree(tree, tree2_min, tree_max);
  OcTree tree_r = *(transformOctree(tree_r_tmp, T));
  auto cloud2i = octreeToPointCloud(tree_r_tmp);
  PointCloud cloud2 = octreeToPointCloud(tree_r);
//  pcl::transformPointCloud(cloud2i, cloud2, T);

<<<<<<< HEAD
  pcl::PointXYZ margin = {1.0, 1.0, 1.0};
  OctreeIntegrationConf conf {100, 0.5, 0.05, margin, 0.001, 0.04 };
=======
  pcl::PointXYZ margin = {0.5, 0.5, 0.5};
  OctreeIntegrationConf conf {100, 0.8, 0.05, margin, 0.001, 0.04 };
>>>>>>> 77a5caf... Package can be compiled now.
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
  viewer.setBackgroundColor(0, 0, 0);
  viewer.addCoordinateSystem(1.0);
  viewer.initCameraParameters();
  viewer.setCameraPosition(0.0, 0.0, 25.0, 0.0, 0.0, 0.0);

  // Visualize first octree as a point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld1 (new pcl::PointCloud<pcl::PointXYZ>);
  *cld1 = octreeToPointCloud(tree_l);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cld1, 0, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ>(cld1, color1, "tree1");
  Vector3f pmin1, pmax1;
  getMinMaxOctree(tree_l, pmin1, pmax1);
  viewer.addCube(pmin1(0), pmax1(0), pmin1(1), pmax1(1), pmin1(2), pmax1(2), 0, 0, 1, "tree1_borders");

  // Visualize second octree as a point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld2 (new pcl::PointCloud<pcl::PointXYZ>);
  *cld2 = octreeToPointCloud(tree_r);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cld2, 255, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ>(cld2, color2, "tree2");
  Vector3f pmin2, pmax2;
  getMinMaxOctree(tree_r, pmin2, pmax2);
  viewer.addCube(pmin2(0), pmax2(0), pmin2(1), pmax2(1), pmin2(2), pmax2(2), 1, 0, 1, "tree2_borders");

  // Transform second tree with estimated transformation and visualize it
  auto transf_tree = transformOctree(tree_l, T_fin);
//  printOcTreeInfo(*transf_tree, "trans_tree");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld3 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(cloud_l, *cld3, T_fin);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cld2, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (cld3, color3, "tree_transformed");
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

TEST(IntegrateOctomaps, EstimationOnly_Demo_PclVis)
{
  auto orig_tree = unpackAndGetOctomap("fr_079");
  printOcTreeInfo(*orig_tree, "original_tree");
  Vector3f o_min, o_max;
  getMinMaxOctree(*orig_tree, o_min, o_max);

  Vector3f tree_min = o_min + Vector3f{25, 8, 0.5};
  Vector3f tree_max = o_max - Vector3f{5, 4, 2.2};
  auto tree = cutOctree(*orig_tree, tree_min, tree_max);

  Vector3f center = (tree_max + tree_min) / 2;
  Vector3f tree1_min = tree_min;
  Vector3f tree1_max = {center(0) + 1.0f, tree_max(1), tree_max(2)};
  Vector3f tree2_min = {center(0) - 1.0f, tree_min(1), tree_min(2)};
  Vector3f tree2_max = tree_max;

  OcTree tree1 = cutOctree(tree, tree1_min, tree1_max);
  OcTree tree2_i = cutOctree(tree, tree2_min, tree2_max);
  printOcTreeInfo(tree2_i, "tree2i");
  auto transf = md::createTransformationMatrix(0.0, 0.2, 0, 0, 0, ToRadians(0));
  OcTree tree2 = *(transformOctree(tree2_i, transf));

  pcl::PointXYZ margin = {1.0, 1.0, 1.0};
  OctreeIntegrationConf conf {100, 0.3, 0.05, margin, 0.001, 0.04 };
  auto T_fin = estimateTransBetweenOctomapsPcl(tree1, tree2, conf);

  printOcTreeInfo(tree, "tree");
  printOcTreeInfo(tree1, "tree1");
  printOcTreeInfo(tree2, "tree2");

  std::cout << "\nApplied transformation: ";
  printTransformation(transf);
  std::cout << "\nEstimated transformation: ";
  printTransformation(T_fin);

#if SHOW_PCL == 1
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters();
  viewer.setCameraPosition(0.0, 0.0, 25.0, 0.0, 0.0, 0.0);

  // Visualize first octree as a point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld1 (new pcl::PointCloud<pcl::PointXYZ>);
  *cld1 = octreeToPointCloud(tree1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color1(cld1, 0, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ> (cld1, color1, "tree1");
  Vector3f pmin1, pmax1;
  getMinMaxOctree(tree1, pmin1, pmax1);
  viewer.addCube(pmin1(0), pmax1(0), pmin1(1), pmax1(1), pmin1(2), pmax1(2), 0, 0, 1, "tree1_borders");

  // Visualize second octree as a point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld2 (new pcl::PointCloud<pcl::PointXYZ>);
  *cld2 = octreeToPointCloud(tree2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color2(cld2, 255, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ> (cld2, color2, "tree2");
  Vector3f pmin2, pmax2;
  getMinMaxOctree(tree2, pmin2, pmax2);
  viewer.addCube(pmin2(0), pmax2(0), pmin2(1), pmax2(1), pmin2(2), pmax2(2), 1, 0, 1, "tree2_borders");


  // Transform second tree with estimated transformation and visualize it
  auto transf_tree = transformOctree(tree1, T_fin);
  printOcTreeInfo(*transf_tree, "trans_tree");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cld3 (new pcl::PointCloud<pcl::PointXYZ>);
  *cld3 = octreeToPointCloud(*transf_tree);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color3(cld2, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (cld3, color3, "tree_transformed");
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

TEST(IntegrateOctomaps, MapsIntegrationDemo)
{
  auto orig_tree = unpackAndGetOctomap("fr_079");
  printOcTreeInfo(*orig_tree, "original_tree");
  Vector3f o_min, o_max;
  getMinMaxOctree(*orig_tree, o_min, o_max);

  Vector3f tree_min = o_min + Vector3f{25, 8, 0.5};
  Vector3f tree_max = o_max - Vector3f{5, 4, 2.2};
  auto tree = cutOctree(*orig_tree, tree_min, tree_max);

  Vector3f center = (tree_max + tree_min) / 2;
  Vector3f tree1_min = tree_min;
  Vector3f tree1_max = {center(0) + 0.5f, tree_max(1), tree_max(2)};
  Vector3f tree2_min = {center(0) - 0.5f, tree_min(1), tree_min(2)};
  Vector3f tree2_max = tree_max;

  OcTree tree1 = cutOctree(tree, tree1_min, tree1_max);
  OcTree tree2_i = cutOctree(tree, tree2_min, tree2_max);
  auto transf = md::createTransformationMatrix(0.1, 0.2, 0, 0, 0, ToRadians(0));
  OcTree tree2 = *(transformOctree(tree2_i, transf));

  printOcTreeInfo(tree, "tree");
  printOcTreeInfo(tree1, "tree1");
  printOcTreeInfo(tree2, "tree2");

  pcl::PointXYZ margin = {0.4, 0.4, 0.05};
  OctreeIntegrationConf conf {1000, 2.0, 0.1, margin, 0.01, 0.05 };

  Matrix4f T_init = Matrix4f::Identity();
  Matrix4f T_fin;
  float error = 0;

  std::cout << "Start integration of octomaps\n";
  auto merged_tree = integrateOctomapsPcl(
      tree1, tree2, conf, T_init, T_fin, error);

  printOcTreeInfo(*merged_tree, "merged tree");
#if RUN_OCTOVIS == 1
  std::string tree_f_path = tmp_path + map_name + "_filtered.ot";
  writeOcTreeToFile(tree, tree_f_path);
  std::system(("octovis " + tree_f_path + "&").c_str());

  std::string tree1_path = tmp_path + map_name + "_p1.ot";
  writeOcTreeToFile(tree1, tree1_path);
  std::system(("octovis " + tree1_path + "&").c_str());

  std::string tree2_path = tmp_path + map_name + "_p2.ot";
  writeOcTreeToFile(tree2, tree2_path);
  std::system(("octovis " + tree2_path + "&").c_str());

  std::string merged_path = tmp_path + map_name + "_m.ot";
  writeOcTreeToFile(*merged_tree, merged_path);
  std::system(("octovis " + merged_path + "&").c_str());

  getchar();
  std::system("killall octovis");
#endif
}

TEST(IntegrateOctomaps, OctomapsIntegrationDemo_2D_EllipsesVis)
{
  // Parameters
  pcl::PointXYZ margin = {100, 100, 1};
  OctreeIntegrationConf conf { 500, 100, 0.1, margin };

  int img_size = 800;
  int cx = img_size / 2;
  int cy = img_size / 2;
  auto img1 = cv::Mat(img_size, img_size, CV_8UC3);

  // Generate common part of maps
  auto common = getEllipsePoints(Vector2f(cx, cy), 50, 100, 0, 200);
  auto e = getEllipsePoints(Vector2f(cx+10, cy+10), 30, 150, kPi/3, 100);
  common = concatenateMatrices(common, e);

  // Generate points set 1
  auto points1 = common;
  auto e11 = getEllipsePoints(Vector2f(cx-200, cy-200), 40, 40, 0, 20);
  points1 = concatenateMatrices(points1, e11);
  auto e12 = getEllipsePoints(Vector2f(cx-100, cy-200), 100, 40, kPi/3, 100);
  points1 = concatenateMatrices(points1, e12);

  // Generate points set 2
  auto points2 = common;
  auto e21 = getEllipsePoints(Vector2f(cx+250, cy+100), 40, 40, 0, 10);
  points2 = concatenateMatrices(points2, e21);
  auto e22 = getEllipsePoints(Vector2f(cx+150, cy+20), 10, 50, 0, 30);
  points2 = concatenateMatrices(points2, e22);
  auto R = md::rotationMatrixFromRPY(0, 0, ToRadians(15.0));
  Vector3f T = {50, 30, 0.0};
  points2 = transformPoints(points2, R, T);

  // Draw points sets and bounds
  drawPoints(img1, points1, CV_RGB(0, 0, 255), 2);
  drawPoints(img1, points2, CV_RGB(255, 0, 0), 2);
  drawPointsRectBounds(img1, points1, CV_RGB(0, 200, 255), 1);
  drawPointsRectBounds(img1, points2, CV_RGB(255, 0, 0), 1);

  OcTree src_tree = PointsToOctree(points1, 1);
  OcTree dst_tree = PointsToOctree(points2, 1);
  Matrix4f T_init = Matrix4f::Identity();
  Matrix4f T_fin;
  float error = 0;

  auto merged_tree = integrateOctomaps(
      src_tree, dst_tree, conf, T_init, T_fin, error);

  auto merged_points = OctreeToPoints(*merged_tree);

  std::cout << "Error = " << error << "\n"
      << "Esimated rotation: "
      << T_fin.block<3,3>(0,0).eulerAngles(0, 1, 2).transpose()
      << "\nEsimated translation: "
      << T_fin.block<3,1>(0,3).transpose() << "\n";

#if SHOW_IMAGES == 1
  auto img2 = cv::Mat(img_size, img_size, CV_8UC3);
  auto cv_white = CV_RGB(255, 255, 255);
  cv::putText(img1, "Two maps with common part",
              cv::Point(cx-150, 25), 0, 0.8, cv_white);
  cv::putText(img2, "Integrated map",
              cv::Point(cx-50, 25), 0, 0.8, cv_white);
  drawPoints(img2, merged_points, CV_RGB(255, 0, 255), 2);
  cv::line(img2, cv::Point(1, 0), cv::Point(1, img2.rows), cv_white, 2);
  auto img3 = concatenateImages(img1, img2);
  cv::line(img3, cv::Point(0, 35), cv::Point(img3.cols, 35), cv_white, 2);
  cv::imshow("im3", img3);
  cv::waitKey();
#endif

  EXPECT_LE(error, 150.0);
}

