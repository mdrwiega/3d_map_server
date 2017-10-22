#include "utils/OctreeUtils.hh"

namespace octomap_tools {

  constexpr unsigned tree_max_value = 32768;
  constexpr unsigned tree_depth_max = 16;

void getNeighborsWithinRadius(const OcTree& tree,
                              const Point& point,
                              const double radiusSquared,
                              const OcTreeNode* node,
                              const OcTreeKey& key,
                              unsigned tree_depth,
                              std::vector<Point>& points,
                              std::vector<float>& distances);

void radiusSearch(const OcTree& tree,
                  const Point &point,
                  const double radius,
                  std::vector<Point> &points,
                  std::vector<float> &distances);

void searchNearestNeighbour(const OcTree& tree,
                            const Point& query,
                            double max_distance,
                            Point& point,
                            float& distance);
}
