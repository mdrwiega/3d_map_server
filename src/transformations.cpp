#include <octomap_tools/transformations.h>


#include <limits>
#include <iomanip>
#include <stdexcept>
#include <algorithm>

#include <octomap_tools/utils.h>
#include <octomap_tools/conversions.h>
#include "md_utils/math/cuboid.h"

#include <pcl/filters/voxel_grid.h>

using namespace octomap;
using namespace Eigen;
using namespace md;

namespace octomap_tools {

template <typename T>
constexpr typename std::enable_if<std::is_arithmetic<T>::value, T>::type
trilinearInterpolation(Vector3<T> p, Vector3<T> p0, Vector3<T> p1,
    T c000, T c001, T c010, T c011, T c100, T c101, T c110, T c111) {

  if (p0(0) == p1(0) || p0(1) == p1(1) || p0(2) == p1(2)) {
    std::cerr << "p0:" << p0.transpose() << "  p1:" << p1.transpose();
    std::runtime_error(std::string(__func__) + ": p0 == p1");
  }

  T xd = (p.x() - p0.x()) / (p1.x() - p0.x());
  T yd = (p.y() - p0.y()) / (p1.y() - p0.y());
  T zd = (p.z() - p0.z()) / (p1.z() - p0.z());

  // x axis
  T c00 = (1 - xd) * c000 + xd * c100;
  T c01 = (1 - xd) * c001 + xd * c101;
  T c10 = (1 - xd) * c010 + xd * c110;
  T c11 = (1 - xd) * c011 + xd * c111;

  // y axis
  T c0 = (1- yd) * c00 + yd * c10;
  T c1 = (1- yd) * c01 + yd * c11;

  // z axis
  return (1 - zd) * c0 + zd * c1;
}

float calculateNewNodeOccupancy(
    const point3d& src_point, const point3d& src_approx_point,
    const OcTree& tree_in, const OcTreeNode* src_node) {
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
  return trilinearInterpolation<float>(p, p0, p1, c000, c001, c010, c011, c100, c101, c110, c111);
}

OcTreePtr transformOctree(const OcTree& tree_in, const Matrix4f& transformation) {
  OcTreePtr tree_out = std::make_unique<OcTree>(tree_in.getResolution());
  Matrix4f inv_transform = transformation.inverse();

  double x_min, x_max, y_min, y_max, z_min, z_max;
  tree_in.getMetricMin(x_min, y_min, z_min);
  tree_in.getMetricMax(x_max, y_max, z_max);
  md::Cuboid box(x_min, x_max, y_min, y_max, z_min, z_max);
  box.transform(transformation);

  Vector3f p_min, p_max;
  box.getMinMax(p_min, p_max);
  double step = tree_in.getResolution();

  for (auto x = p_min(0) - step / 2; x < (p_max(0) + step); x += step) {
    for (auto y = p_min(1) - step / 2; y < (p_max(1) + step); y += step) {
      for (auto z = p_min(2) - step / 2; z < (p_max(2) + step); z += step) {
        Vector4f src_approx_vec = inv_transform * Vector4f(x, y, z, 1);
        point3d src_approx_point(src_approx_vec.x(), src_approx_vec.y(), src_approx_vec.z());
        OcTreeKey src_node_key = tree_in.coordToKey(src_approx_point);
        OcTreeNode* src_node = tree_in.search(src_node_key);

        if (src_node != nullptr) {
          OcTreeKey tgt_node_key = tree_out->coordToKey(point3d(x, y, z));
          point3d src_point = tree_in.keyToCoord(src_node_key);

          auto c = calculateNewNodeOccupancy(src_point, src_approx_point, tree_in, src_node);
          auto new_node = tree_out->updateNode(tgt_node_key, true, true);
          new_node->setLogOdds(logodds(c));
        }
      }
    }
  }

  std::cout << "\nOcTree transformed.\n";
  std::cout << OcTreeInfoToString(tree_in, "input tree");
  std::cout << OcTreeInfoToString(*tree_out, "transformed tree");
  return tree_out;
}

void getMinMaxOctree(const OcTree& tree, Point& min, Point& max) {
  constexpr auto f_min = std::numeric_limits<float>::min();
  constexpr auto f_max = std::numeric_limits<float>::max();
  min = {f_max, f_max, f_max};
  max = {f_min, f_min, f_min};

  // Get min and max of tree
  for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i) {
    const auto& p = i.getCoordinate();
    if (p.x() < min.x) min.x = p.x();
    if (p.y() < min.y) min.y = p.y();
    if (p.z() < min.z) min.z = p.z();
    if (p.x() > max.x) max.x = p.x();
    if (p.y() > max.y) max.y = p.y();
    if (p.z() > max.z) max.z = p.z();
  }
}

void getMinMaxOctree(const OcTree& tree, Eigen::Vector3f& min, Eigen::Vector3f& max) {
  Point pmin, pmax;
  getMinMaxOctree(tree, pmin, pmax);
  min = ToEigen(pmin);
  max = ToEigen(pmax);
}

void filterOutLeafsNotInRange(
    const OcTree& tree_in, const Point& min, const Point& max, OcTree& tree_out) {
  auto pointInRange = [](const Point& point, const Point& rMin, const Point& rMax) {
    return (point.x < rMax.x && point.x > rMin.x) &&
        (point.y < rMax.y && point.y > rMin.y) &&
        (point.z < rMax.z && point.z > rMin.z);
  };

  tree_out.clear();
  tree_out.setResolution(tree_in.getResolution());
  for (auto i = tree_in.begin_leafs(); i != tree_in.end_leafs(); ++i)
  {
    auto p = i.getCoordinate();
    if (pointInRange(ToPcl(p), min, max))
    {
      auto key = i.getKey();
      auto logodds = tree_in.search(key)->getLogOdds();
      tree_out.setNodeValue(p, logodds, true);
    }
  }
}

OcTree cutOctree(const OcTree& tree_in, const Vector3f& min, const Vector3f& max) {
  OcTree tree_out(tree_in.getResolution());
  filterOutLeafsNotInRange(tree_in, ToPcl(min), ToPcl(max), tree_out);
  return tree_out;
}

OcTreePtr CropOcTree(const OcTree& tree_in, const Vector3f& min, const Vector3f& max) {
  OcTreePtr tree_out(new OcTree(tree_in.getResolution()));
  filterOutLeafsNotInRange(tree_in, ToPcl(min), ToPcl(max), *tree_out);
  return tree_out;
}

OcTreePtr sumOctrees(const OcTree& tree1, const OcTree& tree2) {
  OcTreePtr tree_out = std::make_unique<OcTree>(tree1);

  for (auto leaf2 = tree2.begin_leafs(); leaf2 != tree2.end_leafs(); ++leaf2) {
    point3d point = leaf2.getCoordinate();
    OcTreeNode* leaf1 = tree_out->search(point);

    // Node in tree1 not exists. Just simply add it.
    if (leaf1 == nullptr) {
      auto new_node = tree_out->updateNode(point, true);
      new_node->setLogOdds(leaf2->getLogOdds());
    } else {
      int depth1 = getLeafDepth(*tree_out, *leaf1);
      if (depth1 != -1) {
        int depth2 = leaf2.getDepth();
        int depth_diff = depth2 - depth1;
        auto leaf2_logodds = leaf2->getLogOdds();

        // Nodes at the same level
        if (depth_diff == 0) {
          tree_out->updateNodeLogOdds(leaf1, leaf2_logodds);
        }
        else if (depth_diff > 0) { // Node in tree2 is on deeper level than in tree1
          for(int i = 0; i < depth_diff; i++) {
            tree_out->expandNode(leaf1);
            leaf1 = tree_out->search(point);
          }
          tree_out->updateNodeLogOdds(leaf1, leaf2_logodds);
        }
        else if (depth_diff < 0) { // Node in tree1 is on deeper level than in tree2
          for (int i = depth2; i < depth1; i++) {
            OcTreeNode* n = tree_out->search(point, i);
            n->setLogOdds(leaf2_logodds);
            expandNodeOnlyEmptyChilds(n, *tree_out);
          }
          OcTreeNode* n = tree_out->search(point, depth2);
          tree_out->updateNodeLogOdds(n, leaf2_logodds);
        }
      }
    }
  }
  tree_out->prune();
  return tree_out;
}

void extractIntersectingOctrees(
    const OcTree& tree1, const OcTree& tree2,
    const Point& margin,
    OcTree& out_tree1, OcTree& out_tree2) {
  Point min_tree1, max_tree1;
  getMinMaxOctree(tree1, min_tree1, max_tree1);

  auto addPoints = [](const Point& i, const Point& j){
    return Point{i.x + j.x, i.y + j.y, i.z + j.z}; };
  auto subPoints = [](const Point& i, const Point& j){
    return Point{i.x - j.x, i.y - j.y, i.z - j.z}; };

  // Filter out points from tree2 which are not in tree1 range (+ margin)
  min_tree1 = subPoints(min_tree1, margin);
  max_tree1 = addPoints(max_tree1, margin);
  filterOutLeafsNotInRange(tree2, min_tree1, max_tree1, out_tree2);

  // Filtered tree 2 range
  Point min_tree2, max_tree2;
  getMinMaxOctree(out_tree2, min_tree2, max_tree2);

  // Filter out points from tree1 which are not in tree2 filtered range (+ margin)
  min_tree2 = subPoints(min_tree2, margin);
  max_tree2 = addPoints(max_tree2, margin);
  filterOutLeafsNotInRange(tree1, min_tree2, max_tree2, out_tree1);
}

void extractIntersectingAndDownsamplePointClouds(
    const PointCloud& cloud1, const PointCloud& cloud2,
    float voxelSize, const Point& margin,
    PointCloud& cloud1reduced, PointCloud& cloud2reduced) {
  PointCloud::Ptr cloud1filtered(new PointCloud);
  PointCloud::Ptr cloud2filtered(new PointCloud);

  Point minCloud1, maxCloud1;
  pcl::getMinMax3D(cloud1, minCloud1, maxCloud1);
  filterOutPointsNotInRange(cloud2, minCloud1, maxCloud1, *cloud2filtered);

  Point minCloud2filtered, maxCloud2filtered;
  pcl::getMinMax3D(*cloud2filtered, minCloud2filtered, maxCloud2filtered);

  auto addPoints = [](const Point& i, const Point& j){
    return Point{i.x+j.x, i.y+j.y, i.z+j.z}; };
  auto subPoints = [](const Point& i, const Point& j){
    return Point{i.x-j.x, i.y-j.y, i.z-j.z}; };

  minCloud2filtered = subPoints(minCloud2filtered, margin);
  maxCloud2filtered = addPoints(maxCloud2filtered, margin);

  filterOutPointsNotInRange(cloud1, minCloud2filtered, maxCloud2filtered, *cloud1filtered);

  pcl::VoxelGrid<Point> grid;
  grid.setLeafSize(voxelSize, voxelSize, voxelSize);
  grid.setInputCloud(cloud1filtered);
  grid.filter(cloud1reduced);

  grid.setLeafSize(voxelSize, voxelSize, voxelSize);
  grid.setInputCloud(cloud2filtered);
  grid.filter(cloud2reduced);

  Point minCloud2, maxCloud2;
  pcl::getMinMax3D(cloud2, minCloud2, maxCloud2);
  Point min_red1, max_red1;
  pcl::getMinMax3D(cloud1reduced, min_red1, max_red1);
  Point min_red2, max_red2;
  pcl::getMinMax3D(cloud1reduced, min_red2, max_red2);

//  LOG_DBG() << "Extract intersecting\n"
//      << "Input cloud 1 size: " << cloud1.size()
//      << " min: " << minCloud1 << " max: " << maxCloud1 << std::endl
//      << "Input cloud 2 size: " << cloud2.size()
//      << " min: " << minCloud2 << " max: " << maxCloud2 << std::endl
//      << "Filtered cloud 1 size: " << cloud1filtered->size() << std::endl
//      << "Filtered cloud 2 size: " << cloud2filtered->size() << std::endl
//      << "Downsampled cloud 1 size: " << cloud1reduced.size()
//      << " min: " << min_red1 << " max: " << max_red1 << std::endl
//      << "Downsampled cloud 2 size: " << cloud2reduced.size()
//      << " min: " << min_red2 << " max: " << max_red2 << std::endl;
}

}
