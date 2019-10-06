/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <octomap_tools/utils.h>

#include <iomanip>

namespace octomap_tools {

PointCloud createUniformPointCloud(Point min, Point max, Point step)
{
  if (min.x > max.x || min.y > max.y || min.z > max.z)
    throw std::runtime_error("Incorrect ranges");

  PointCloud cloud;

  for (float i = min.x; i < max.x; i += step.x)
    for (float j = min.y; j < max.y; j += step.y)
      for (float k = min.z; k < max.z; k += step.z)
        cloud.push_back(Point{(float)i, (float)j, (float)k});

  return cloud;
}

int getNodeDepth(const OcTree& tree, const octomap::point3d& point,
                 const OcTreeNode& node)
{
  for(int depth = tree.getTreeDepth(); depth > 1; --depth)
  {
    if (tree.search(point, depth) == &node)
      return depth;
  }
  return -1;
}

int getLeafDepth(const OcTree& tree, const OcTreeNode& node)
{
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it)
  {
    if (tree.search(it.getIndexKey()) == &node)
      return static_cast<int>(it.getDepth());
  }
  return -1;
}


void filterOutPointsNotInRange(const PointCloud& cloudIn,
                               const Point& min, const Point& max,
                               PointCloud& cloudOut)
{
  auto pointInRange = [](const Point& point, const Point& rMin, const Point& rMax){
    return (point.x < rMax.x && point.x > rMin.x) &&
        (point.y < rMax.y && point.y > rMin.y) &&
        (point.z < rMax.z && point.z > rMin.z);
  };

  cloudOut.clear();
  for (const auto& point : cloudIn)
  {
    if (pointInRange(point, min, max))
      cloudOut.push_back(point);
  }
}

void expandNodeOnlyEmptyChilds(OcTreeNode* node, OcTree& tree) {
  for (unsigned k = 0; k < 8; k++) {
    if (!node->childExists(k)) {
      OcTreeNode* newNode = tree.createNodeChild(node, k);
      newNode->copyData(*node);
    }
  }
}

void printPointsAndDistances(std::string title, std::vector<Point>& points,
                             std::vector<float>& distances)
{
  std::cout << title << ": Points and distances: \n";
  for (unsigned i = 0; i < points.size(); ++i)
    std::cout << "(" << points[i].x << ", " << points[i].y << ", "
    << points[i].z << ") = " << distances[i] << "\n";
}

int getKeyDepth(const OcTree& tree, const octomap::point3d& point,
                const octomap::OcTreeKey& key)
{
  for (int depth = tree.getTreeDepth(); depth > 1; --depth)
  {
    if (tree.coordToKey(point, depth) == key)
      return depth;
  }
  return -1;
}

bool contains(OcTree& tree, const Point& query, float sqRadius,
              const octomap::OcTreeKey& o)
{
  auto p = tree.keyToCoord(o);
  auto x = std::abs(query.x - p.x());
  auto y = std::abs(query.y - p.y());
  auto z = std::abs(query.z - p.z());
  auto depth = getKeyDepth(tree, p, o);
  auto half_size = tree.getNodeSize(depth) / 2;
  x += half_size;
  y += half_size;
  z += half_size;

  Point pp(x,y,z);
  return squaredNorm(pp) < sqRadius;
}

// compute which child is closest to the query point
int getClosestChild(const Point& q, const Point& p)
{
  return ((q.x - p.x) >= 0) |
      (((q.y - p.y) >= 0) << 1) |
      (((q.z - p.z) >= 0) << 2);
}

float squaredNorm(const Point& p)
{
  return p.x * p.x + p.y * p.y + p.z * p.z;
}

float squaredDistance(const Point& p, const Point& q)
{
  Point x(p.x - q.x, p.y - q.y, p.z - q.z);
  return squaredNorm(x);
}

double getVoxelSquaredSideLen(const OcTree& tree, unsigned tree_depth_arg)
{
  // side length of the voxel cube increases exponentially with the octree depth
  double side_len = tree.getResolution() * static_cast<double>(1 << (tree.getTreeDepth() - tree_depth_arg));
  return side_len * side_len;
}

double getVoxelSquaredDiameter(const OcTree& tree, unsigned tree_depth_arg)
{
  // return the squared side length of the voxel cube as a function of the octree depth
  return getVoxelSquaredSideLen(tree, tree_depth_arg) * 3;
}

}
