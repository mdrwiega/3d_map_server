#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#define EXPECT_POINT3D_EQ(n1, n2) \
    EXPECT_NEAR(n1.x(), n2.x(), 1e-5); \
    EXPECT_NEAR(n1.y(), n2.y(), 1e-5); \
    EXPECT_NEAR(n1.z(), n2.z(), 1e-5);

constexpr double kPi  = 3.14159265358979323846;

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
