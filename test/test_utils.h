#pragma once

#include <type_traits>
#include <algorithm>
#include <limits>
#include <cmath>

#include "stdlib.h"
#include "stdio.h"
#include "string.h"

#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <octomap_tools/utils.h>
#include <octomap_tools/transformations.h>
#include <octomap_tools/math.h>

using namespace Eigen;
using namespace octomap;
using Vector3f = Eigen::Vector3f;

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

inline int GetVirtualMemoryUsedByProcessKB() {
  FILE* file = fopen("/proc/self/status", "r");
  int result = -1;
  char line[128];

  auto parseLine = [](char* line) {
    int i = strlen(line);
    const char* p = line;
    while (*p <'0' || *p > '9') p++;
    line[i-3] = '\0';
    i = atoi(p);
    return i;
  };

  while (fgets(line, 128, file) != NULL) {
    if (strncmp(line, "VmSize:", 7) == 0) {
      result = parseLine(line);
      break;
    }
  }
  fclose(file);
  return result;
}

inline std::string MatchingTestResultToString(
  const Eigen::Matrix4f& t1, const Eigen::Matrix4f& t2, float fitness_score) {
  std::stringstream ss;
  ss << "\nReal transformation:\n" << transfMatrixToXyzRpyString(t1);
  ss << "Estimated transformation:\n" << transfMatrixToXyzRpyString(t2);
  ss << "Fitness score: " << fitness_score << "\n";
  ss << "Real error: " << transformationsError(t1, t2) << "\n";
  return ss.str();
}

inline void PrintMatchingResult(const Eigen::Matrix4f& t1, const Eigen::Matrix4f& t2, float fitness_score) {
  std::cout << MatchingTestResultToString(t1, t2, fitness_score);
}

inline void checkIfTransformedTreeBoundsAreCorrect(
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

}

