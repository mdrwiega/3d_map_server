/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <iostream>
#include <iomanip>
#include <sstream>

namespace octomap_tools {

template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

class Cuboid {
 public:
  Cuboid() = default;

  Cuboid(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax) {
    using namespace Eigen;
    vertices[0] = Vector3f(xMax, yMin, zMin); vertices[1] = Vector3f(xMin, yMin, zMin);
    vertices[2] = Vector3f(xMin, yMax, zMin); vertices[3] = Vector3f(xMax, yMax, zMin);
    vertices[4] = Vector3f(xMax, yMin, zMax); vertices[5] = Vector3f(xMin, yMin, zMax);
    vertices[6] = Vector3f(xMin, yMax, zMax); vertices[7] = Vector3f(xMax, yMax, zMax);
  }

  Eigen::Vector3f& operator [](unsigned vertexNr) {
    if (vertexNr >= vertices.size())
      throw std::out_of_range(std::string(__func__) + ": Incorrect index");
    return vertices[vertexNr];
  }

  void transform(Eigen::Matrix4f transform) {
    for (auto& v : vertices) {
      Eigen::Vector4f point(v.x(), v.y(), v.z(), 1);
      point = transform * point;
      v = Eigen::Vector3f(point.x(), point.y(), point.z());
    }
  }

  void getMinMax(Eigen::Vector3f& pMin, Eigen::Vector3f& pMax) {
    for (std::size_t i = 0; i < 3; ++i) {
      auto result = std::minmax_element(vertices.cbegin(), vertices.cend(),
                                        [&i](auto& a, auto& b) { return a(i) < b(i); });
      pMin(i) = (*result.first)(i);
      pMax(i) = (*result.second)(i);
    }
  }
 private:
  std::array<Eigen::Vector3f, 8> vertices;
};

inline Eigen::Matrix3f rotationMatrixFromRPY(float roll, float pitch, float yaw) {
  using namespace Eigen;
  Matrix3f m;
  m = AngleAxisf(roll,  Vector3f::UnitX())
    * AngleAxisf(pitch, Vector3f::UnitY())
    * AngleAxisf(yaw,   Vector3f::UnitZ());
  return m;
}

inline Eigen::Matrix4f transformationMat(const Eigen::Matrix3f& R, const Eigen::Vector3f& T) {
  Eigen::Matrix4f transform;
  transform.block<3,3>(0,0) = R;
  transform.block<3,1>(0,3) = T;
  transform(3,3) = 1;
  return transform;
}

inline Eigen::Matrix4f createTransformationMatrix(float x, float y, float z, float roll, float pitch, float yaw) {
  const auto R = rotationMatrixFromRPY(roll, pitch, yaw);
  Eigen::Vector3f T = {x, y, z};
  return transformationMat(R, T);
}

inline Eigen::Matrix4f inverseTransform(const Eigen::Matrix4f& transform) {
  Eigen::Matrix4f T_inv;
  Eigen::Matrix3f R = transform.block<3,3>(0,0).transpose();

  T_inv.block<3,3>(0,0) = R;
  T_inv.block<3,1>(0,3) = - (R * transform.block<3,1>(0,3));
  T_inv.block<1,4>(3,0) = transform.block<1,4>(3,0);
  return T_inv;
}

inline std::string transformationMatrixToString(const Eigen::Matrix4f& Mat) {
  Eigen::Matrix3f rot = Mat.block<3,3>(0, 0);
  Eigen::Vector3f transl = Mat.block<3,1>(0, 3);
  std::stringstream ss;
  ss << "\n" << std::setprecision(3) << std::fixed;
  ss << "    | " << rot(0,0) << ' ' << rot(0,1) << ' ' << rot(0,2) << " | \n";
  ss << "R = | " << rot(1,0) << ' ' << rot(1,1) << ' ' << rot(1,2) << " | \n";
  ss << "    | " << rot(2,0) << ' ' << rot(2,1) << ' ' << rot(2,2) << " | \n";
  ss << "\n";
  ss << "t = < " << transl(0) << ' ' << transl(1) << ' ' << transl(2) << "\n";
  return ss.str();
}

inline float transoformationsError(const Eigen::Matrix4f& t1, const Eigen::Matrix4f& t2) {
  return (t1.inverse() * t2 - Eigen::Matrix4f::Identity()).norm();
}

inline Eigen::Vector3f rotMatrixToRPY(const Eigen::Matrix3f& rot) {
  return rot.eulerAngles(0, 1, 2).transpose();
}

inline std::string transfMatrixToXyzRpyString(const Eigen::Matrix4f& Mat) {
  Eigen::Matrix3f rot_mat = Mat.block<3,3>(0, 0);
  Eigen::Vector3f rot = rotMatrixToRPY(rot_mat);
  Eigen::Vector3f transl = Mat.block<3,1>(0, 3);
  std::stringstream ss;
  ss << "\n" << std::setprecision(3) << std::fixed;
  ss << "XYZ = " << transl(0) << ' ' << transl(1) << ' ' << transl(2) << "\n";
  ss << "RPY = " << rot(0) << ' ' << rot(1) << ' ' << rot(2) << "\n";
  return ss.str();
}

}
