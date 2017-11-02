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
using namespace cv;

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

  auto image1 = cvCreateImage(cvSize(img_size, img_size), 8, 3);
  cvZero(image1);

  // Generate points set 1
  auto points1 = generateEllipsePoints(Vector2f(cx, cy), 50, 100, 0, 200);
  auto e1 = generateEllipsePoints(Vector2f(cx-200, cy-200), 40, 40, 0, 20);
  points1 = concatenateMatrices(points1, e1);
  auto e11 = generateEllipsePoints(Vector2f(cx-100, cy-200), 100, 40, kPi/3, 100);
  points1 = concatenateMatrices(points1, e11);

  // Draw bounds of points 1
  auto p1_min = points1.rowwise().minCoeff();
  auto p1_max = points1.rowwise().maxCoeff();
  auto p0 = cvPoint(p1_min(0,0), p1_min(1,0));
  auto p1 = cvPoint(p1_max(0,0), p1_max(1,0));
  cvDrawRect(image1, p0, p1, CV_RGB(0, 200, 255), 2);
  drawPoints(image1, points1, CV_RGB(0, 0, 255), 2);

  // Generate points set 2
  auto points2 = generateEllipsePoints(
      Vector2f(cx + T(0), cy + T(1)), 50, 100, ToRadians(angle), 200);
  auto e2 = generateEllipsePoints(Vector2f(cx+250, cy+100), 40, 40, 0, 10);
  points2 = concatenateMatrices(points2, e2);

  // Draw bounds of points 2
  auto p2_min = points2.rowwise().minCoeff();
  auto p2_max = points2.rowwise().maxCoeff();
  auto p20 = cvPoint(p2_min(0,0), p2_min(1,0));
  auto p21 = cvPoint(p2_max(0,0), p2_max(1,0));
  cvDrawRect(image1, p20, p21, CV_RGB(255, 200, 0), 2);
  drawPoints(image1, points2, CV_RGB(255, 0, 0), 2);

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
  auto image2 = cvCreateImage(cvSize(img_size, img_size), 8, 3);
  cvZero(image2);
  drawPoints(image2, merged_points, CV_RGB(255, 0, 255), 2);
  cvDrawLine(image2, cvPoint(1, 0), cvPoint(1, img_size), CV_RGB(200, 200, 200), 2);
  auto im1 = cv::cvarrToMat(image1);
  auto im2 = cv::cvarrToMat(image2);
  auto im3 = concatenateImages(im1, im2);
  imshow("im3", im3);
  cvWaitKey();
#endif

  cvReleaseImage(&image1);
  //  EXPECT_LE(error, 65.0);
}

