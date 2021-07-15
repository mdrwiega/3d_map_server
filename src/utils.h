#pragma once

#include <memory>
#include <iomanip>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <ros/package.h>
#include <ros/console.h>

#include <octomap/octomap.h>
#include <Eigen/Dense>

#include <octomap_tools/types.h>
#include <octomap_tools/octomap_io.h>

namespace octomap_tools {

inline bool exists(const std::string& file_path) {
  std::ifstream f(file_path.c_str());
  return f.good();
}

inline OcTreePtr unpackAndGetOctomap(
    const std::string& map_name, const std::string& ext = "ot") {
  const std::string tmp_path = "tmp/";
  const std::string ds_path = ros::package::getPath("3d_map_server") + "/octomaps_dataset/";
  const std::string map_path = tmp_path + map_name + "." + ext;

  std::system(("rm -rf " + tmp_path).c_str());
  std::system(("mkdir -p " + tmp_path).c_str());
  std::string map_packed_path = ds_path + map_name + "." + ext + ".gz";
  std::system(("gzip -cd " + map_packed_path + " > " + map_path).c_str());
  std::cout << "Unpacked file: " << map_packed_path
            << " to " << map_path << std::endl;

  auto tree = LoadOcTreeFromFile(map_path);
  return tree;
}

inline std::string getCurrentDateAndTime() {
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  return oss.str();
}

inline PointCloud createUniformPointCloud(Point min, Point max, Point step) {
  if (min.x > max.x || min.y > max.y || min.z > max.z)
    throw std::runtime_error("Incorrect ranges");

  PointCloud cloud;

  int i_max = (max.x - min.x) / step.x;
  int j_max = (max.y - min.y) / step.y;
  int k_max = (max.z - min.z) / step.z;

  for (int i = 0; i < i_max; ++i) {
    float x = min.x + step.x * i;
    for (int j = 0; j < j_max; ++j) {
      float y = min.y + step.y * j;
      for (int k = 0; k < k_max; ++k) {
        float z = min.z + step.z * k;
        cloud.push_back(Point{x, y, z});
      }
    }
  }
  return cloud;
}

inline int getNodeDepth(const OcTree& tree, const octomap::point3d& point, const OcTreeNode& node) {
  for(int depth = tree.getTreeDepth(); depth > 1; --depth)
  {
    if (tree.search(point, depth) == &node)
      return depth;
  }
  return -1;
}

inline int getLeafDepth(const OcTree& tree, const OcTreeNode& node) {
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    if (tree.search(it.getIndexKey()) == &node)
      return static_cast<int>(it.getDepth());
  }
  return -1;
}

inline int getNumberOfOccupiedNodes(const OcTree& input_tree) {
  octomap::OcTree tree = input_tree;
  tree.expand();
  int num_occupied = 0;
  for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i) {
     if (tree.isNodeOccupied(*i))
       num_occupied++;
   }
  return num_occupied;
}

inline std::string OcTreeInfoToString(const OcTree& tree, const std::string& name,
                                      const std::string& line_prefix = {}) {
  std::stringstream ss;
  ss << line_prefix << name << ":\n";
  ss << line_prefix << "  size: " << tree.size() << "\n";
  ss << line_prefix << "  resolution: " << tree.getResolution() << "\n";

  if (tree.size() > 0) {
    ss << "  leafs: " << tree.getNumLeafNodes() << "\n";
    ss << "  occupied_nodes: " << getNumberOfOccupiedNodes(tree) << "\n";

    double x_min, x_max, y_min, y_max, z_min, z_max;
    tree.getMetricMin(x_min, y_min, z_min);
    tree.getMetricMax(x_max, y_max, z_max);
    ss << "  limits:\n";
    ss << "    x: [" << x_min << ", " << x_max << "]\n";
    ss << "    y: [" << y_min << ", " << y_max << "]\n";
    ss << "    z: [" << z_min << ", " << z_max << "]\n";
  }
  return ss.str();
}

inline void PrintOcTreeInfo(const OcTree& input_tree, const std::string& name) {
  std::cout << OcTreeInfoToString(input_tree, name);
}

inline void printOcTree(const OcTree& tree, const std::string& name) {
  PrintOcTreeInfo(tree, name);
  std::cout << "Leafs:\n";
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
    std::cout << it.getCoordinate() << "  LogOdds: " << it->getLogOdds()
                                        << "  Depth: " << getLeafDepth(tree, *it) << "\n";
  }
  std::cout << std::endl;
}

// Get min and max of tree in O(log n) where n is number of tree nodes
inline void getMinMaxOctree(const OcTree& tree, Point& min, Point& max) {
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

inline void getMinMaxOctree(const OcTree& tree, Eigen::Vector3f& min, Eigen::Vector3f& max) {
  Point pmin, pmax;
  getMinMaxOctree(tree, pmin, pmax);
  min = Eigen::Vector3f{pmin.x, pmin.y, pmin.z};
  max = Eigen::Vector3f{pmax.x, pmax.y, pmax.z};
}

inline void filterOutPointsNotInRange(const PointCloud& cloudIn,
                               const Point& min, const Point& max,
                               PointCloud& cloudOut) {
  auto point_in_range = [](const Point& point, const Point& rMin, const Point& rMax){
    return (point.x < rMax.x && point.x > rMin.x) &&
        (point.y < rMax.y && point.y > rMin.y) &&
        (point.z < rMax.z && point.z > rMin.z);
  };

  cloudOut.clear();
  for (const auto& point : cloudIn) {
    if (point_in_range(point, min, max))
      cloudOut.push_back(point);
  }
}

inline void expandNodeOnlyEmptyChilds(OcTreeNode* node, OcTree& tree) {
  for (unsigned k = 0; k < 8; k++) {
    if (!tree.nodeChildExists(node, k)) {
      OcTreeNode* new_node = tree.createNodeChild(node, k);
      new_node->copyData(*node);
    }
  }
}

inline void printPointsAndDistances(const std::string& title, std::vector<Point>& points,
                             std::vector<float>& distances) {
  std::cout << title << ": Points and distances: \n";
  for (unsigned i = 0; i < points.size(); ++i) {
    std::cout << "(" << points[i].x << ", " << points[i].y << ", "
    << points[i].z << ") = " << distances[i] << "\n";
  }
}

inline int getKeyDepth(const OcTree& tree, const octomap::point3d& point, const octomap::OcTreeKey& key) {
  for (int depth = tree.getTreeDepth(); depth > 1; --depth) {
    if (tree.coordToKey(point, depth) == key) {
      return depth;
    }
  }
  return -1;
}

inline float squaredNorm(const Point& p) {
  return p.x * p.x + p.y * p.y + p.z * p.z;
}

inline float squaredDistance(const Point& p, const Point& q) {
  Point x(p.x - q.x, p.y - q.y, p.z - q.z);
  return squaredNorm(x);
}

inline bool contains(OcTree& tree, const Point& query, float sqRadius,
              const octomap::OcTreeKey& o) {
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
inline int getClosestChild(const Point& q, const Point& p) {
  return (static_cast<int>((q.x - p.x) >= 0.0) |
         (static_cast<int>((q.y - p.y) >= 0.0) << 1) |
         (static_cast<int>((q.z - p.z) >= 0.0) << 2));
}

inline double getVoxelSquaredSideLen(const OcTree& tree, unsigned tree_depth_arg)
{
  // side length of the voxel cube increases exponentially with the octree depth
  double side_len = tree.getResolution() * static_cast<double>(1 << (tree.getTreeDepth() - tree_depth_arg));
  return side_len * side_len;
}

inline double getVoxelSquaredDiameter(const OcTree& tree, unsigned tree_depth_arg)
{
  // return the squared side length of the voxel cube as a function of the octree depth
  return getVoxelSquaredSideLen(tree, tree_depth_arg) * 3;
}

inline void downsamplePointCloud(const PointCloud::ConstPtr& cloudIn,
                                 PointCloud& cloudOut, double voxelSize) {
  pcl::VoxelGrid<Point> grid;
  grid.setLeafSize(voxelSize, voxelSize, voxelSize);
  grid.setInputCloud(cloudIn);
  grid.filter(cloudOut);
}

/**
 * Splits pointcloud into two parts
 *
 * @param[in] plane - plane equation in form: a*x + b*y + c*y + d = 0
 *                    where elements of vector are factors: a,b,c,d
 * @param[in] in - input pointcloud
 * @param[out] out1 - output pointcloud which contains points bigger than plane
 * @param[out] out2 - output pointcloud which contains points smaller than plane
 */
inline void splitPointcloud(const Eigen::Vector4f& plane,
                            const PointCloud& in, PointCloud& out1, PointCloud& out2) {
  out1.clear();
  out2.clear();

  for (const auto& p : in.points) {
    if ((plane[0] * p.x + plane[1] * p.y + plane[2] * p.z + plane[3]) > 0) {
      out1.push_back(p);
    }
    else {
      out2.push_back(p);
    }
  }
}

inline std::string PointCloudInfoToString(
    const PointCloud& cloud, const std::string& cloud_name) {
  std::stringstream ss;
  Point min, max;
  pcl::getMinMax3D(cloud, min, max);

  ss << "\nPointcloud: " << cloud_name
     << "\nSize: " << cloud.size()
     << std::setprecision(3)
     << "\nLimits: x(" << min.x << ", " << max.x << ")  "
     << "y(" << min.y << ", " << max.y << ")  "
     << "z(" << min.z << ", " << max.z << ")\n";
  return ss.str();
}

inline void ExpandOccupiedNodesRecursive(octomap::OcTree& tree,
                                         octomap::OcTreeNode* node,
                                         unsigned depth) {
  if (depth >= tree.getTreeDepth()) {
    return;
  }

  if (node == nullptr) {
    return;
  }

  // Do not expand if node is not occupied
  if (!tree.isNodeOccupied(node)) {
    return;
  }
  // Expand node if it doesn't have children
  if (!tree.nodeHasChildren(node)) {
    tree.expandNode(node);
  }

  // Recursively expand children
  for (unsigned int i=0; i<8; i++) {
    if (tree.nodeChildExists(node, i)) {
      ExpandOccupiedNodesRecursive(tree, tree.getNodeChild(node, i), depth+1);
    }
  }
}

inline int GetNumberOfNaNInPointCloud(const PointCloud &cloud) {
    int j = 0;
    for (int i = 0; i < static_cast<int>(cloud.points.size ()); ++i) {
      if (!std::isfinite (cloud.points[i].x) ||
          !std::isfinite (cloud.points[i].y) ||
          !std::isfinite (cloud.points[i].z)) {
            j++;
          }
    }
    return j;
}

inline void FilterOutNaNs(PointCloud::Ptr& cloud_ptr, bool debug = false) {
  // Find points with NaNs
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  for (size_t i = 0; i < cloud_ptr->size(); ++i) {
    if (!std::isfinite(cloud_ptr->points[i].x) ||
        !std::isfinite(cloud_ptr->points[i].y) ||
        !std::isfinite(cloud_ptr->points[i].z)) {
      inliers->indices.push_back(i);
    }
  }

  // Filter out points with NANs
  if (!inliers->indices.empty()) {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_ptr);
    if (debug) {
      std::cout << "\nRemoved " << inliers->indices.size() << " points with NaN\n";
    }
  }
}

} // namespace octomap_tools