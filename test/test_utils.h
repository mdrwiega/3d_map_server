#pragma once

#include <cmath>
#include <unistd.h>

#include <Eigen/Dense>
#include <opencv/highgui.h>
#include <opencv/cv.hpp>
#include "utils/types_conversions.h"
#include "utils/octree_utils.h"

#define EXPECT_POINT3D_EQ(n1, n2) \
    EXPECT_NEAR(n1.x(), n2.x(), 1e-5); \
    EXPECT_NEAR(n1.y(), n2.y(), 1e-5); \
    EXPECT_NEAR(n1.z(), n2.z(), 1e-5);

namespace octomap_tools {

static const std::string ds_path = "../datasets/";
static const std::string tmp_path = "tmp/";

inline bool isFileExist(const std::string& file_name)
{
    std::ifstream infile(file_name.c_str());
    return infile.good();
}

inline std::unique_ptr<OcTree> unpackAndGetOctomap(
    const std::string& map_name, const std::string ext = "ot")
{
  const std::string map_path = tmp_path + map_name + "." + ext;

  if (!isFileExist(map_path))
  {
    std::system(("rm -rf " + tmp_path).c_str());
    std::system(("mkdir -p " + tmp_path).c_str());
    std::string map_packed_path = ds_path + map_name + "." + ext + ".gz";
    std::system(("gzip -cd " + map_packed_path + " > " + map_path).c_str());
    std::cout << "Unpacked file: " << map_packed_path
              << " to " << map_path << std::endl;
  }

  return readOctreeFromFile(map_path);
}

inline Eigen::Matrix3Xf getEllipsePoints(
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

inline Eigen::Matrix3Xf transformPoints(
    const Eigen::Matrix3Xf& points,
    const Eigen::Matrix3f& R, const Eigen::Vector3f& T)
{
  Eigen::Matrix3Xf points_out(3, points.cols());
  for (auto i = 0; i < points.cols(); ++i)
    points_out.col(i) = R * points.col(i) + T;
  return points_out;
}

inline void drawPoints(cv::Mat& image, const Eigen::Matrix3Xf& points,
                CvScalar color = CV_RGB(255,255,255), int thickness = 1)
{
  for (unsigned i = 0; i < points.cols(); ++i)
  {
    auto point_cv = cv::Point((int)points(0,i), (int)points(1,i));
    cv::circle(image, point_cv, thickness, color, 1);
  }
}

inline void drawPointsRectBounds(
    cv::Mat& img, const Eigen::Matrix3Xf& points,
    CvScalar color = CV_RGB(255,255,255), int thickness = 1)
{
  auto pmin = points.rowwise().minCoeff();
  auto pmax = points.rowwise().maxCoeff();
  cv::rectangle(img, ToCv(pmin), ToCv(pmax), color, thickness);

}

inline Eigen::Matrix3Xf concatenateMatrices(const Eigen::Matrix3Xf& A,
                                     const Eigen::Matrix3Xf& B)
{
  Eigen::Matrix3Xf C(3, A.cols() + B.cols());
  C << A, B;
  return C;
}

inline cv::Mat concatenateImages(const cv::Mat& im1, const cv::Mat& im2)
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

}

