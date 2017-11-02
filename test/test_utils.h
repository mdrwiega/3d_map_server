#pragma once

#include <cmath>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#define EXPECT_POINT3D_EQ(n1, n2) \
    EXPECT_NEAR(n1.x(), n2.x(), 1e-5); \
    EXPECT_NEAR(n1.y(), n2.y(), 1e-5); \
    EXPECT_NEAR(n1.z(), n2.z(), 1e-5);

constexpr double kPi  = 3.14159265358979323846;

inline float ToRadians(float deg)
{
  return deg * kPi / 180.0;
}

inline Eigen::Matrix3Xf generateEllipsePoints(
    Eigen::Vector2f s, float a, float b,
    float rot_angle, unsigned points_num)
{
  using namespace Eigen;

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

inline void drawPoints(IplImage* image, const Eigen::Matrix3Xf& points,
                CvScalar color = CV_RGB(255,255,255), int thickness = 1)
{
  for (unsigned i = 0; i < points.cols(); ++i)
  {
    auto point_cv = cvPoint((int)points(0,i), (int)points(1,i));
    cvDrawCircle(image, point_cv, thickness, color, 1);
  }
}

inline octomap::OcTree PointsToOctree(const Eigen::Matrix3Xf& points, double tree_resolution)
{
  octomap::OcTree tree(tree_resolution);
  for (auto i = 0; i < points.cols(); ++i)
  {
    auto point = octomap::point3d{points(0,i), points(1,i), points(2,i)};
    tree.setNodeValue(point, 1.0, true);
  }

  return tree;
}

inline Eigen::Matrix3Xf OctreeToPoints(const octomap::OcTree& tree)
{
  Eigen::Matrix3Xf m(3, tree.getNumLeafNodes());
  int k = 0;
  // Traverse all leafs in the tree
  for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i)
  {
    m.col(k++) = Eigen::Vector3f(i.getX(), i.getY(), i.getZ());
  }
  return m;
}

Eigen::Matrix3Xf concatenateMatrices(Eigen::Matrix3Xf& A, Eigen::Matrix3Xf& B)
{
  Eigen::Matrix3Xf C(3, A.cols() + B.cols());
  C << A, B;
  return C;
}

cv::Mat concatenateImages(cv::Mat& im1, cv::Mat& im2)
{
  auto sz1 = im1.size();
  auto sz2 = im2.size();
  cv::Mat im3(sz1.height, sz1.width + sz2.width, CV_8UC3);
  cv::Mat left(im3, cv::Rect(0, 0, sz1.width, sz1.height));
  im1.copyTo(left);
  cv::Mat right(im3, cv::Rect(sz1.width, 0, sz2.width, sz2.height));
  im2.copyTo(right);
  return im3;
}

inline Eigen::Matrix4f transformationMat(const Eigen::Matrix3f& R, const Eigen::Vector3f& T)
{
  Eigen::Matrix4f transform;
  transform.block<3,3>(0,0) = R;
  transform.block<3,1>(0,3) = T;
  transform(3,3) = 1;
  return transform;
}

