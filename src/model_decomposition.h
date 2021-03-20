#pragma once

#include <Eigen/Dense>

namespace octomap_tools {

inline std::vector<Eigen::Vector2f> generateSpiralCoordinatesSequence(int size_x, int size_y);
inline std::vector<Rectangle> generateBlocksInSpiralOrder(
    Eigen::Vector2f& min, Eigen::Vector2f& max, Eigen::Vector2f& step_xy);

/**
* Model decomposition into rectangular blocks.
* Returns vector of rectangles which defines submodels coordinations.
*/
std::vector<Rectangle> RectangularModelDecomposition(
  const Eigen::Vector4f& map_min, const Eigen::Vector4f& map_max,
  float block_size_x, float block_size_y) {

  Eigen::Vector2f rectangle_min(map_min.x(), map_min.y());
  Eigen::Vector2f rectangle_max(map_max.x(), map_max.y());
  Eigen::Vector2f step_xy(block_size_x, block_size_y);

  std::cout << "Generation of cells for rectangle "
      << "min: (" << rectangle_min(0) << ", " << rectangle_min(1) << ")  "
      << "max: (" << rectangle_max(0) << ", " << rectangle_max(1) << ")";

  return generateBlocksInSpiralOrder(rectangle_min, rectangle_max, step_xy);
}

std::vector<Eigen::Vector2f> generateSpiralCoordinatesSequence(int size_x, int size_y) {
  std::vector<Eigen::Vector2f> seq;
  int i;
  int k = 0; // start row
  int l = 0; // start column
  int m = size_x; // end row
  int n = size_y; // end column

  while (k < m && l < n) {
      // First row from the remaining rows
      for (i = l; i < n; ++i) {
        seq.push_back(Eigen::Vector2f(k, i));
      }
      k++;

      // Last column from the remaining columns
      for (i = k; i < m; ++i) {
          seq.push_back(Eigen::Vector2f(i, n-1));
      }
      n--;

      // Last row from the remaining rows
      if ( k < m) {
          for (i = n-1; i >= l; --i) {
            seq.push_back(Eigen::Vector2f(m-1, i));
          }
          m--;
      }

      // First column from the remaining columns
      if (l < n) {
          for (i = m-1; i >= k; --i) {
            seq.push_back(Eigen::Vector2f(i, l));
          }
          l++;
      }
  }
  return seq;
}

std::vector<Rectangle> generateBlocksInSpiralOrder(
    Eigen::Vector2f& min, Eigen::Vector2f& max, Eigen::Vector2f& step_xy) {
  std::vector<Rectangle> cells;
  int size_x = ceil((max(0) - min(0)) / step_xy(0)) ;
  int size_y = ceil((max(1) - min(1)) / step_xy(1)) ;
  std::vector<Eigen::Vector2f> seq = generateSpiralCoordinatesSequence(size_x, size_y);

  for (auto& i : seq) {
    Rectangle rect;
    rect.min(0) = min(0) + i(0) * step_xy(0);
    rect.min(1) = min(1) + i(1) * step_xy(1);
    rect.max(0) = min(0) + (i(0) + 1) * step_xy(0);
    rect.max(1) = min(1) + (i(1) + 1) * step_xy(1);

    if (rect.max(0) > max(0)) rect.max(0) = max(0);
    if (rect.max(1) > max(1)) rect.max(1) = max(1);

    cells.push_back(rect);
  }
  return cells;
}

} // namespace octomap_tools
