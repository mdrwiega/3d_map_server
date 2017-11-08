/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include "test_utils.h"
#include "octomap_integrator.h"
#include "md_utils/math/transformations.h"
#include "utils/octree_utils.h"
#include "octree_transformations.h"

using namespace Eigen;
using namespace octomap_tools;
using namespace octomap;
using namespace md;

#define SHOW_IMAGES 0
#define RUN_OCTOVIS 0

static const std::string ds_path = "datasets/";
static const std::string tmp_path = "build/tmp/";

std::unique_ptr<OcTree> unpackAndGetOctomap(
    const std::string& map_name, const std::string ext = "ot")
{
  const std::string map_path = tmp_path + map_name + "." + ext;

  std::system(("rm -rf " + tmp_path).c_str());
  std::system(("mkdir -p " + tmp_path).c_str());
  std::system(("gzip -cd " + ds_path + map_name +
      "." + ext + ".gz > " + map_path).c_str());

  return readOctreeFromFile(map_path);
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

