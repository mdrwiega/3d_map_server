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

inline Eigen::Matrix3f rotationMatrixFromRPY(
    float roll, float pitch, float yaw)
{
  using namespace Eigen;
  Matrix3f m;
  m = AngleAxisf(roll,  Vector3f::UnitX())
    * AngleAxisf(pitch, Vector3f::UnitY())
    * AngleAxisf(yaw,   Vector3f::UnitZ());
  return m;
}

inline Eigen::Matrix4f transformationMat(
    const Eigen::Matrix3f& R, const Eigen::Vector3f& T)
{
  Eigen::Matrix4f transform;
  transform.block<3,3>(0,0) = R;
  transform.block<3,1>(0,3) = T;
  transform(3,3) = 1;
  return transform;
}

inline Eigen::Matrix4f createTransformationMatrix(
    float x, float y, float z, float roll, float pitch, float yaw)
{
  const auto R = rotationMatrixFromRPY(roll, pitch, yaw);
  Eigen::Vector3f T = {x, y, z};
  return transformationMat(R, T);
}

inline Eigen::Matrix4f inverseTransform(const Eigen::Matrix4f& transform)
{
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

inline float transoformationsError(
    const Eigen::Matrix4f& t1, const Eigen::Matrix4f& t2)
{
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
