#pragma once

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <stdexcept>
#include <iomanip>
#include <sstream>

namespace octomap_tools {

template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

class Cuboid {
 public:
  Cuboid() = default;

  Cuboid(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max) {
    using namespace Eigen;
    vertices[0] = Vector3f(x_max, y_min, z_min); vertices[1] = Vector3f(x_min, y_min, z_min);
    vertices[2] = Vector3f(x_min, y_max, z_min); vertices[3] = Vector3f(x_max, y_max, z_min);
    vertices[4] = Vector3f(x_max, y_min, z_max); vertices[5] = Vector3f(x_min, y_min, z_max);
    vertices[6] = Vector3f(x_min, y_max, z_max); vertices[7] = Vector3f(x_max, y_max, z_max);
  }

  Eigen::Vector3f& operator [](unsigned vertexNr) {
    if (vertexNr >= vertices.size()) {
      throw std::out_of_range(std::string(__func__) + ": Incorrect index");
    }
    return vertices[vertexNr];
  }

  void Transform(const Eigen::Matrix4f& transform) {
    for (auto& v : vertices) {
      Eigen::Vector4f point(v.x(), v.y(), v.z(), 1);
      point = transform * point;
      v = Eigen::Vector3f(point.x(), point.y(), point.z());
    }
  }

  void GetMinMax(Eigen::Vector3f& pMin, Eigen::Vector3f& pMax) {
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
  const auto r = rotationMatrixFromRPY(roll, pitch, yaw);
  Eigen::Vector3f t = {x, y, z};
  return transformationMat(r, t);
}

inline Eigen::Matrix4f inverseTransform(const Eigen::Matrix4f& transform) {
  Eigen::Matrix4f inv;
  Eigen::Matrix3f rot = transform.block<3,3>(0,0).transpose();

  inv.block<3,3>(0,0) = rot;
  inv.block<3,1>(0,3) = - (rot * transform.block<3,1>(0,3));
  inv.block<1,4>(3,0) = transform.block<1,4>(3,0);
  return inv;
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

inline double transformationsError(const Eigen::Matrix4f& t1, const Eigen::Matrix4f& t2) {
  return (t1 * t2 - Eigen::Matrix4f::Identity()).norm();
}

inline Eigen::Vector3f rotMatrixToRPY(const Eigen::Matrix3f& rot) {
  return rot.eulerAngles(0, 1, 2).transpose();
}

inline std::string transfMatrixToXyzRpyString(const Eigen::Matrix4f& transf, const std::string line_prefix = {}) {
  Eigen::Matrix3f rot_mat = transf.block<3,3>(0, 0);
  Eigen::Vector3f rot = rotMatrixToRPY(rot_mat);
  Eigen::Vector3f transl = transf.block<3,1>(0, 3);
  std::stringstream ss;
  ss << std::setprecision(3) << std::fixed;
  ss << line_prefix << "xyz: [" << transl(0) << ", " << transl(1) << ", " <<  transl(2) << "]\n";
  ss << line_prefix << "rpy: [" <<  rot(0) << ", " <<  rot(1) << ", " <<  rot(2) << "]\n";
  return ss.str();
}

template<typename T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
near(T x, T y, int units_last_place = 5) {
    return std::abs(x-y) <= std::numeric_limits<T>::epsilon() * std::abs(x+y) * units_last_place
        || std::abs(x-y) < std::numeric_limits<T>::min();
}

template<typename T>
bool near_abs(T x, T y, T abs_error) {
    return std::abs(x-y) <= abs_error;
}

} // namespace octomap_tools