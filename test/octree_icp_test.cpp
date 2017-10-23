/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "utils/OctreeUtils.hh"
#include "utils/Logger.hh"

#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>

#include "md_utils/math/transformations.hh"
#include "octree_icp.h"

#include <opencv2/opencv.hpp>

using namespace Eigen;
using namespace octomap_tools;
using namespace md_utils;
using namespace octomap;

constexpr double kPi  = 3.14159265358979323846;

#define SHOW_IMAGES 1

Matrix3Xf generateEllipsePoints(Vector2f s, float a, float b,
                                float rot_angle, unsigned points_num)
{
  float d = 0;
  Matrix3Xf points(3, points_num);

  for (unsigned i = 0; i < points_num; ++i)
  {
    Vector2f p = { a * cos(d), b * sin(d) };
    p = Rotation2Df(rot_angle) * p + s;
    points.col(i) = Vector3f(p(0,0), p(1,0), 0);
    d += 2 * kPi / points_num;
  }
  return points;
}

void drawPoints(IplImage* image, const Matrix3Xf& points,
                CvScalar color = CV_RGB(255,255,255), int thickness = 1)
{
  for (unsigned i = 0; i < points.cols(); ++i)
  {
    auto point_cv = cvPoint((int)points(0,i), (int)points(1,i));
    cvDrawCircle(image, point_cv, thickness, color, 1);
  }
}

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

OcTree PointsToOctree(const Matrix3Xf& points, double tree_resolution)
{
  OcTree tree(tree_resolution);
  for (auto i = 0; i < points.cols(); ++i)
  {
    auto point = octomap::point3d{points(0,i), points(1,i), points(2,i)};
    tree.setNodeValue(point, 1.0, true);
  }

  return tree;
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
