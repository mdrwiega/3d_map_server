/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "utils/OctreeUtils.hh"

#include <gtest/gtest.h>
#include "octree_icp.h"
#include "test_utils.h"

using namespace Eigen;
using namespace octomap_tools;
using namespace octomap;

#define SHOW_IMAGES 0

TEST(OctreeIcpTest, ellipsesVisualization_IcpWithOcTreeNearestNeighbours)
{
  auto image = cvCreateImage(cvSize(500, 500), 8, 3);
  cvZero(image);
  int num_points = 200;

  // Create destination ellipse - model
  auto dst_points = generateEllipsePoints(
      Vector2f(250, 200), 100, 200, kPi / 3, num_points);
  drawPoints(image, dst_points, CV_RGB(255, 0, 0), 2);

  // Create source ellipse
  auto src_points = generateEllipsePoints(
      Vector2f(180, 300), 110, 190, kPi / 10, num_points / 4);
  drawPoints(image, src_points, CV_RGB(0, 0, 255), 2);

  Matrix3f R;
  Vector3f T;
  float error = icp(src_points, dst_points, R, T,
                    50, 0.001, nearestNeighboursOcTree);

  std::cout << "Error = " << error << "\n";
  std::cout << "Final transformation :\n";
  std::cout << "Rotation: \n" << R
            << "\nTranslation:\n " << T.transpose() << "\n";

  // Print transformed ellipse
  for(auto i = 0; i < src_points.cols() ; i++ )
    src_points.col(i) = R * src_points.col(i) + T;
  drawPoints(image, src_points, CV_RGB(255, 0, 255), 5);

  #if SHOW_IMAGES == 1
    cvShowImage("image",image);
    cvWaitKey();
  #endif

  cvReleaseImage(&image);
  EXPECT_LE(error, 65.0);
}

TEST(OctreeIcpTest, IcpWithOcTreeNearestNeighbours)
{
  auto image = cvCreateImage(cvSize(500, 500), 8, 3);
  cvZero(image);
  int num_points = 200;

  // Create destination ellipse - model
  auto dst_points = generateEllipsePoints(
      Vector2f(250, 200), 100, 200, kPi / 3, num_points);
  drawPoints(image, dst_points, CV_RGB(255, 0, 0), 2);

  // Create source ellipse
  auto src_points = generateEllipsePoints(
      Vector2f(180, 300), 110, 190, kPi / 10, num_points / 4);
  drawPoints(image, src_points, CV_RGB(0, 0, 255), 2);

  Matrix3f R;
  Vector3f T;
  OcTree src_tree = PointsToOctree(src_points, 0.5);
  OcTree dst_tree = PointsToOctree(dst_points, 0.5);
  float error = icp(src_tree, dst_tree, R, T, 50, 0.001);

  std::cout << "Error = " << error << "\n";
  std::cout << "Final transformation :\n";
  std::cout << "Rotation: \n" << R
            << "\nTranslation:\n " << T.transpose() << "\n";

  // Print transformed ellipse
  for(auto i = 0; i < src_points.cols() ; i++ )
    src_points.col(i) = R * src_points.col(i) + T;
  drawPoints(image, src_points, CV_RGB(255, 0, 255), 5);

  #if SHOW_IMAGES == 1
    cvShowImage("image",image);
    cvWaitKey();
  #endif

  cvReleaseImage(&image);
  EXPECT_LE(error, 65.0);
}
