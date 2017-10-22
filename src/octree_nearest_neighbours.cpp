#include "octree_nearest_neighbours.h"
#include "utils/Logger.hh"

using namespace octomap;

namespace octomap_tools {

void getNeighborsWithinRadius(const OcTree& tree,
                              const Point& point,
                              const double radiusSquared,
                              const OcTreeNode* node,
                              const OcTreeKey& key,
                              unsigned tree_depth,
                              std::vector<Point>& points,
                              std::vector<float>& distances)
{
  // std::cout << "--------- getNN ------------------------------\n"
  //      << "point: (" << point.x << ", " << point.y << ", " << point.z << ")  "
  //      << "depth: " << tree_depth << "   radiusSq=" << radiusSquared
  //      << "  voxel_sq_diam: " << voxel_squared_diameter << "\n";

  if (!tree.nodeHasChildren(node)) // Is leaf
  {
    auto c = tree.keyToCoord(key, tree_depth);
    Point node_center(c.x(), c.y(), c.z());
    float dist_sq = squaredDistance(node_center, point);

    if (dist_sq <=  radiusSquared)
    {
      points.push_back(node_center);
      distances.push_back(std::sqrt(dist_sq));
    }
    return;
  }

  for (unsigned child_id = 0; child_id < 8; child_id++)
  {
    if (!tree.nodeChildExists(node, child_id))
      continue;

    OcTreeKey child_key;
    key_type t = tree_max_value >> (tree_depth + 1);
    computeChildKey(child_id, t, key, child_key);

    point3d c = tree.keyToCoord(child_key, tree_depth + 1);
    Point voxel_center(c.x(), c.y(), c.z());
    float squared_dist = squaredDistance(voxel_center, point);
    float diameter_sq =  getVoxelSquaredDiameter(tree, tree_depth);

    //    std::cout << "Child: " << child_idx << "  center: ("
    //              << c.x() << ", " << c.y() << ", " << c.z()
    //              << ")  sqDist: " << squared_dist << "\n";
    if (squared_dist <= diameter_sq / 4.0 + radiusSquared
                      + sqrt(diameter_sq * radiusSquared))
    {
      const OcTreeNode* child = tree.getNodeChild(node, child_id);
      getNeighborsWithinRadius(tree, point, radiusSquared,
                               child, child_key,
                               tree_depth + 1, points, distances);
    }
  }
}

void radiusSearch(const OcTree& tree, const Point &point,
                  const double radius,
                  std::vector<Point>& points,
                  std::vector<float>& distances)
{
  OcTreeKey root_key;
  root_key[0] = root_key[1] = root_key[2] = tree_max_value;
  getNeighborsWithinRadius(tree, point, radius * radius,
                                    tree.getRoot(), root_key, 0,
                                    points, distances);
}

void searchNearestNeighbour(const OcTree& tree, const Point& query,
                            double max_distance, Point& point, float& distance)
{
  std::vector<Point> points;
  std::vector<float> dists;
  radiusSearch(tree, query, max_distance, points, dists);

  auto i = std::min_element(dists.begin(),dists.end()) - dists.begin();
  point = points[i];
  distance = dists[i];
}

}
