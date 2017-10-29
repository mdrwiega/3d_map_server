#include "octree_transformations.h"

#include "utils/Logger.hh"

using namespace octomap;
using namespace Eigen;
using namespace md_utils;

namespace octomap_tools {

OcTreePtr transformOctree(const OcTree& tree_in,
                          const Matrix4f& transformation)
{
  OcTreePtr tree_out = std::make_unique<OcTree>(tree_in.getResolution());
  Matrix4f inv_transform = inverseTransform(transformation);

  double x_min, x_max, y_min, y_max, z_min, z_max;
  tree_in.getMetricMin(x_min, y_min, z_min);
  tree_in.getMetricMax(x_max, y_max, z_max);
  md_utils::Cuboid box(x_min, x_max, y_min, y_max, z_min, z_max);
  box.transform(transformation);

  Vector3f p_min, p_max;
  box.getMinMax(p_min, p_max);
  double step = tree_in.getResolution();

  // Traverse octree nodes at lowest level (leafs)
  for (auto x = p_min(0) - step / 2; x < (p_max(0) + step); x += step)
  {
    for (auto y = p_min(1) - step / 2; y < (p_max(1) + step); y += step)
    {
      for (auto z = p_min(2) - step / 2; z < (p_max(2) + step); z += step)
      {
        OcTreeKey tgt_node_key = tree_out->coordToKey(point3d(x, y, z));
        Vector4f src_approx_point = inv_transform * Vector4f(x, y, z, 1);
        OcTreeKey src_node_key = tree_in.coordToKey(
            point3d(src_approx_point(0), src_approx_point(1), src_approx_point(2)));

        point3d src_point = tree_in.keyToCoord(src_node_key);
        OcTreeNode* src_node = tree_in.search(src_node_key);

        if (src_node != nullptr)
        {
          point3d approx_point(src_approx_point.x(), src_approx_point.y(), src_approx_point.z());
          auto c = calculateNewNodeOccupancy(src_point, approx_point, tree_in, src_node);
          auto new_node = tree_out->updateNode(tgt_node_key, true);
          new_node->setLogOdds(logodds(c));
        }
      }
    }
  }
  return tree_out;
}

float calculateNewNodeOccupancy(
    const point3d& src_point, const point3d& src_approx_point,
    const OcTree& tree_in, const OcTreeNode* src_node)
{
  auto sign = [](auto i) { return (i < 0) ? -1 : 1; };
  const double tree_res = tree_in.getResolution();
  auto x0 = src_point.x();
  auto x1 = src_point.x() + sign(src_approx_point.x() - src_point.x()) * tree_res;
  auto y0 = src_point.y();
  auto y1 = src_point.y() + sign(src_approx_point.y() - src_point.y()) * tree_res;
  auto z0 = src_point.z();
  auto z1 = src_point.z() + sign(src_approx_point.z() - src_point.z()) * tree_res;

  // Values used to trilinear interpolation
  float c000 = src_node->getOccupancy();
  float c001 = 0, c010 = 0, c011 = 0, c100 = 0, c101 = 0, c110 = 0, c111 = 0;

  OcTreeNode* node;
  if ((node = tree_in.search(point3d(x0, y0, z1))) != nullptr)
    c001 = node->getOccupancy();
  if ((node = tree_in.search(point3d(x0, y1, z0))) != nullptr)
    c010 = node->getOccupancy();
  if ((node = tree_in.search(point3d(x0, y1, z1))) != nullptr)
    c011 = node->getOccupancy();
  if ((node = tree_in.search(point3d(x1, y0, z0))) != nullptr)
    c100 = node->getOccupancy();
  if ((node = tree_in.search(point3d(x1, y1, z1))) != nullptr)
    c101 = node->getOccupancy();
  if ((node = tree_in.search(point3d(x1, y1, z0))) != nullptr)
    c110 = node->getOccupancy();
  if ((node = tree_in.search(point3d(x1, y1, z1))) != nullptr)
    c111 = node->getOccupancy();

  auto p  = Vector3<float>(src_approx_point.x(), src_approx_point.y(), src_approx_point.z());
  auto p0 = Vector3<float>(x0, y0, z0);
  auto p1 = Vector3<float>(x1, y1, z1);
  return trilinearInterpolation<float>(
      p, p0, p1, c000, c001, c010, c011, c100, c101, c110, c111);
}

void extractIntersectingOctrees(
    const OcTree& tree1, const OcTree& tree2,
    const Point& margin,
    OcTree& out_tree1, OcTree& out_tree2)
{

}

}
