#include "octree_nearest_neighbours.h"

#include "kdtree/kdtree.h"
#include "utils/logger.h"
#include "utils/types_conversions.h"

using namespace octomap;
using namespace Eigen;

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

  if (dists.size() > 0)
  {
    auto i = std::min_element(dists.begin(), dists.end()) - dists.begin();
    point = points[i];
    distance = dists[i];
  }
  else
  {
    point = query;
    distance = 0;
//    std::cout << "NN Point not found for ("
//        << query.x << ", " << query.y << ", " << query.z << "\n";
  }
}

void nearestNeighboursKdTree(const Eigen::Matrix3Xf& dst_points,
                             Eigen::Matrix3Xf& src_points,
                             Eigen::Matrix3Xf& nearest_neighbours)
{
  struct kdtree *ptree = kd_create(3);

  for (unsigned i = 0; i < dst_points.cols(); i++ )
  {
    Vector3f v = dst_points.col(i);
    kd_insertf((struct kdtree*) ptree, (float*)&v, 0);
  }

  for (unsigned i = 0; i < src_points.cols(); i++)
  {
    Vector3f v = src_points.col(i);
    struct kdres* results = kd_nearestf((struct kdtree *)ptree, (float*)&v);
    kd_res_end(results);
    Vector3f nn;
    kd_res_itemf(results, (float*)&nn);
    kd_res_free(results);
    nearest_neighbours.col(i) = nn;
  }
  kd_free(ptree);
}

void nearestNeighboursOcTree(const Eigen::Matrix3Xf& dst_points,
                             Eigen::Matrix3Xf& src_points,
                             Eigen::Matrix3Xf& nearest_neighbours)
{
  constexpr double kMaxDist = 200.0;
  PointCloud cloud;
  for (auto i = 0; i < dst_points.cols(); ++i)
    cloud.push_back(ToPcl(dst_points.col(i)));
  OcTree tree = PointCloudToOctree(cloud, 0.5);

  for (unsigned i = 0; i < src_points.cols(); i++)
  {
    Point point_q = ToPcl(src_points.col(i));
    float dist;
    Point nn;
    searchNearestNeighbour(tree, point_q, kMaxDist, nn, dist);
    nearest_neighbours.col(i) = ToEigen(nn);
  }
}

void nearestNeighboursOnOcTree(const OcTree& tree_dst,
                             Eigen::Matrix3Xf& src_points,
                             Eigen::Matrix3Xf& nearest_neighbours)
{
  constexpr double kMaxDist = 200.0;

  for (unsigned i = 0; i < src_points.cols(); i++)
  {
    Point point_q = ToPcl(src_points.col(i));
    float dist;
    Point nn;
    searchNearestNeighbour(tree_dst, point_q, kMaxDist, nn, dist);
    nearest_neighbours.col(i) = ToEigen(nn);
  }
}

}
