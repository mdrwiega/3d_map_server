/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "utils/OctreeUtils.hh"

#include <gtest/gtest.h>
#include "octree_icp.h"
#include "test_utils.h"
#include "md_utils/math/transformations.hh"
#include "octree_transformations.h"
#include "octomap_merger.h"

using namespace Eigen;
using namespace octomap_tools;
using namespace octomap;
using namespace md_utils;

#define SHOW_IMAGES 1

TEST(IntegrateOctomaps, OctomapsIntegrationDemo_2D_EllipsesVis)
{
  // Parameters
  float angle = 15.0;
  Vector3f T = {60, 20, 0.0};
  unsigned max_iter = 500;
  pcl::PointXYZ margin = {100, 100, 1};

  int img_size = 800;
  int cx = img_size / 2;
  int cy = img_size / 2;

  auto img1 = cv::Mat(img_size, img_size, CV_8UC3);

  // Generate points set 1
  auto points1 = generateEllipsePoints(Vector2f(cx, cy), 50, 100, 0, 200);
  auto e1 = generateEllipsePoints(Vector2f(cx-200, cy-200), 40, 40, 0, 20);
  points1 = concatenateMatrices(points1, e1);
  auto e11 = generateEllipsePoints(Vector2f(cx-100, cy-200), 100, 40, kPi/3, 100);
  points1 = concatenateMatrices(points1, e11);

  // Draw bounds of points 1
  auto p1_min = points1.rowwise().minCoeff();
  auto p1_max = points1.rowwise().maxCoeff();
  cv::rectangle(img1, ToCv(p1_min), ToCv(p1_max), CV_RGB(0, 200, 255), 2);
  drawPoints(img1, points1, CV_RGB(0, 0, 255), 2);

  // Generate points set 2
  auto points2 = generateEllipsePoints(
      Vector2f(cx + T(0), cy + T(1)), 50, 100, ToRadians(angle), 200);
  auto e2 = generateEllipsePoints(Vector2f(cx+250, cy+100), 40, 40, 0, 10);
  points2 = concatenateMatrices(points2, e2);

  // Draw bounds of points 2
  auto p2_min = points2.rowwise().minCoeff();
  auto p2_max = points2.rowwise().maxCoeff();
  cv::rectangle(img1, ToCv(p2_min), ToCv(p2_max), CV_RGB(255, 200, 0), 2);
  drawPoints(img1, points2, CV_RGB(255, 0, 0), 2);

  OcTree src_tree = PointsToOctree(points1, 1);
  OcTree dst_tree = PointsToOctree(points2, 1);
  OcTree src_tree_f(1);
  OcTree dst_tree_f(1);

  extractIntersectingOctrees(
      src_tree, dst_tree, margin, src_tree_f, dst_tree_f);

  Matrix3f R_est;
  Vector3f T_est;
  float error = icp(src_tree_f, dst_tree_f, R_est, T_est, max_iter, 0.001);

  std::cout << "Error = " << error << "\n";
  std::cout << "Final transformation :\n";
  std::cout << "Rotation: \n" << R_est
      << "\nTranslation:\n " << T_est.transpose() << "\n";

  auto trans = transformationMat(R_est, T_est);
  auto trans_src_tree = transformOctree(src_tree, trans);
  auto merged_tree = sumOctrees(*trans_src_tree, dst_tree);
  auto merged_points = OctreeToPoints(*merged_tree);

#if SHOW_IMAGES == 1
  auto img2 = cv::Mat(img_size, img_size, CV_8UC3);
  drawPoints(img2, merged_points, CV_RGB(255, 0, 255), 2);
  cv::line(img2, cv::Point(1, 0), cv::Point(1, img_size), CV_RGB(200, 200, 200), 2);
  auto img3 = concatenateImages(img1, img2);
  cv::imshow("im3", img3);
  cv::waitKey();
#endif

  //  EXPECT_LE(error, 65.0);
}

