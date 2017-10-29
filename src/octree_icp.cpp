#include "octree_icp.h"

#include <exception>
#include <limits>
#include "octree_nearest_neighbours.h"

using namespace Eigen;

namespace octomap_tools {

/**
 * Calculates the least squares best fit transformation
 * This transformation maps corresponding points A to B in m spatial dimensions
 *
 * @param A Nxm matrix of corresponding points
 * @param B Nxm matrix of corresponding points
 * @param R mxm rotation matrix
 * @param T mx1 translation vector
 */
void leastSquaresBestFitTransform(const Matrix3Xf& A, const Matrix3Xf& B,
                               Matrix3f& R, Vector3f& T)
{
  if (A.cols() != B.cols() || A.rows() != B.rows())
    throw std::runtime_error(std::string(__func__) + ": Incorrect A,B size.");

  unsigned m = A.cols();
  Vector3f centroid_A(Vector3f::Zero());
  Vector3f centroid_B(Vector3f::Zero());

  // Calculate centroids
  for (unsigned i = 0; i < A.cols(); ++i)
  {
    centroid_A += A.col(i);
    centroid_B += B.col(i);
  }
  centroid_A /= A.cols();
  centroid_B /= B.cols();

  // Translate points to their centroids
  Matrix3Xf AA(3, m);
  Matrix3Xf BB(3, m);
  for (unsigned i = 0; i < m; ++i)
  {
    AA.col(i) = A.col(i) - centroid_A;
    BB.col(i) = B.col(i) - centroid_B;
  }

  // SVD
  Matrix3f HH = AA * BB.transpose();
  JacobiSVD<Matrix3f> svd(HH, ComputeFullU | ComputeFullV);

  // Rotation matrix
  R = svd.matrixV() * (svd.matrixU().transpose());

  // Special reflection case
  if (R.determinant() < 0)
  {
    Matrix3f V = svd.matrixV();
    V.col(2) = V.col(2) * -1;
    R = V * (svd.matrixU().transpose());
  }

  // Transformation matrix
  T = - R * centroid_A + centroid_B;
}

inline float squaredDistance(const Eigen::Vector3f& a,
                             const Eigen::Vector3f& b)
{
  return (a - b).squaredNorm();
}

float icp(const Matrix3Xf& new_points, const Matrix3Xf& dst_points,
          Matrix3f& R, Vector3f& T,
          unsigned max_iter, float tolerance,
          NearestNeighboursFcn nnCalculator)
{
  float err = 0;
  float prev_err = std::numeric_limits<float>::max();
  Matrix3Xf src_points = new_points;

  for (unsigned k = 0; k < max_iter; ++k)
  {
    // Finds nearest neighbors between current source and destination points
    Matrix3Xf nearest_neighbours(3, new_points.cols());
    nnCalculator(dst_points, src_points, nearest_neighbours);

    // Calculate error value;
    err = 0.0;
    for (auto i = 0; i < src_points.cols(); ++i)
      err += squaredDistance(src_points.col(i),
                             nearest_neighbours.col(i));

    // Calculate the transformation between source and nearest destination points
    leastSquaresBestFitTransform(src_points, nearest_neighbours, R, T);

    // Update the current source set
    for(auto i = 0; i < src_points.cols() ; i++ )
      src_points.col(i) = R * src_points.col(i) + T;

    if (fabs(err - prev_err) < std::pow(tolerance, 2))
      break;
    prev_err = err;
  }
  // Final transformation
  leastSquaresBestFitTransform(new_points, src_points, R, T);

  return sqrt(err);
}

Matrix3Xf treeLeafsToPoints(const OcTree& tree)
{
  Matrix3Xf points(3, tree.getNumLeafNodes());
  unsigned i = 0;
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it)
  {
    auto p = it.getCoordinate();
    points.col(i++) = Vector3f(p.x(), p.y(), p.z());
  }
  return points;
}

float icp(const OcTree& src_tree, const OcTree& dst_tree,
          Matrix3f& R, Vector3f& T,
          unsigned max_iter, float tolerance)
{
  float err = 0;
  float prev_err = std::numeric_limits<float>::max();
  auto src_points = treeLeafsToPoints(src_tree);

  for (unsigned k = 0; k < max_iter; ++k)
  {
    // Finds nearest neighbors between current source and destination points
    Matrix3Xf nearest_neighbours(3, src_tree.getNumLeafNodes());
    nearestNeighboursOnOcTree(dst_tree, src_points, nearest_neighbours);

    // Calculate error value;
    err = 0.0;
    for (auto i = 0; i < src_points.cols(); ++i)
      err += squaredDistance(src_points.col(i), nearest_neighbours.col(i));

    // Calculate the transformation between source and nearest destination points
    leastSquaresBestFitTransform(src_points, nearest_neighbours, R, T);

    // Update the current source set
    for(auto i = 0; i < src_points.cols() ; i++ )
      src_points.col(i) = R * src_points.col(i) + T;

    if (fabs(err - prev_err) < std::pow(tolerance, 2))
      break;
    prev_err = err;
  }
  // Final transformation
  auto new_points = treeLeafsToPoints(src_tree);
  leastSquaresBestFitTransform(new_points, src_points, R, T);

  return sqrt(err);
}

}
