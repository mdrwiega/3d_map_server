#pragma once

#include <type_traits>
#include <algorithm>
#include <limits>
#include <cmath>

#include <ros/package.h>

#include <Eigen/Dense>

#define EXPECT_POINT3D_EQ(n1, n2) \
    EXPECT_NEAR(n1.x(), n2.x(), 1e-5); \
    EXPECT_NEAR(n1.y(), n2.y(), 1e-5); \
    EXPECT_NEAR(n1.z(), n2.z(), 1e-5);

#define EXPECT_MATRIX_EQ(m1, m2)           \
    for (int i = 0; i < m1.rows(); ++i)    \
    for (int j = 0; j < m1.cols(); ++j)    \
    EXPECT_NEAR(m1(i,j), m2(i,j), 1e-6);

#define EXPECT_VECTOR3F_NEAR(n1, n2, val) \
    EXPECT_NEAR(n1[0], n2[0], val); \
    EXPECT_NEAR(n1[1], n2[1], val); \
    EXPECT_NEAR(n1[2], n2[2], val)

namespace octomap_tools {

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

void checkIfTransformedTreeBoundsAreCorrect(
    const OcTree& tree, const OcTree& transf_tree, const Eigen::Matrix4f& transf) {
  Vector3f min1, max1;
  getMinMaxOctree(tree, min1, max1);
  Vector3f min2, max2;
  getMinMaxOctree(transf_tree, min2, max2);
  //  printOcTreeInfo(tree, "Tree");
  //  printOcTreeInfo(transf_tree, "Transformed tree");

  auto R = transf.block<3,3>(0,0);
  auto T = transf.block<3,1>(0,3);

  Vector3f min1t = R * min1 + T;
  Vector3f max1t = R * max1 + T;

  double max_err = tree.getResolution() * std::sqrt(3);
  EXPECT_VECTOR3F_NEAR(min1t, min2, max_err);
  EXPECT_VECTOR3F_NEAR(max1t, max2, max_err);

}

inline bool isFileExist(const std::string& file_name) {
    std::ifstream infile(file_name.c_str());
    return infile.good();
}

inline std::unique_ptr<OcTree> unpackAndGetOctomap(
    const std::string& map_name, const std::string ext = "ot") {
  const std::string tmp_path = "tmp/";
  const std::string ds_path = ros::package::getPath("octomap_tools") + "/octomaps_dataset/";
  const std::string map_path = tmp_path + map_name + "." + ext;

  std::system(("rm -rf " + tmp_path).c_str());
  std::system(("mkdir -p " + tmp_path).c_str());
  std::string map_packed_path = ds_path + map_name + "." + ext + ".gz";
  std::system(("gzip -cd " + map_packed_path + " > " + map_path).c_str());
  std::cout << "Unpacked file: " << map_packed_path
            << " to " << map_path << std::endl;

  auto tree = readOctreeFromFile(map_path);
  printOcTreeInfo(*tree, "Loaded tree");
  return tree;
}

}

