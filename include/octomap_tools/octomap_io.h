#pragma once

#include <memory>
#include <stdexcept>

#include <ros/console.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <octomap/octomap.h>

#include <octomap_tools/types.h>

namespace octomap_tools {

inline OcTreePtr LoadOcTreeFromFile(const std::string& filename) {
  ROS_INFO_STREAM("Loading octree from file: " << filename);
  std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);
  if (!file.is_open()) {
    throw std::runtime_error(std::string(__func__) + ": Unable to open " + filename);
  }

  std::istream::pos_type streampos = file.tellg();
  OcTreePtr tree;

  // Reading new format of octree (.ot)
  if (filename.length() > 3 && (filename.compare(filename.length()-3, 3, ".ot") == 0)) {
    tree.reset(dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(file)));
    if (!tree) {
      file.clear();
      file.seekg(streampos);
      throw std::runtime_error(std::string(__func__) + ": Can't load octree from file " + filename);
    }
  } else { // Reading old format of octree (.bt)
    std::unique_ptr<octomap::OcTree> b_tree (new octomap::OcTree(0.1));

    if (b_tree->readBinary(file) && b_tree->size() > 1) {
      tree.reset(b_tree.release());
    } else {
      throw std::runtime_error(std::string(__func__) + ": Can't detect binary OcTree format in file " + filename);
    }
  }
  return tree;
}

inline void SaveOcTreeToFile(const OcTree& tree, const std::string& filename) {
  ROS_INFO_STREAM("Saving octree to file: " << filename);
  if (!tree.write(filename)) {
    throw std::runtime_error(std::string(__func__) + ": Error during saving to " + filename);
  }
}

// [[depracated("Lossy conversion from cloud to octree")]]
inline void SavePointCloudAsOctreeToFile(
    PointCloud::Ptr& cloud, const std::string& fileName, float resolution) {
  octomap::Pointcloud scan;

  for (const auto& i : cloud->points)
    scan.push_back(i.x, i.y, i.z);

  std::unique_ptr<octomap::OcTree> tree(new octomap::OcTree(resolution));

  octomap::point3d sensor_pose{0,0,0};
  tree->insertPointCloud(scan, sensor_pose);
  tree->updateInnerOccupancy();
  tree->prune();

  SaveOcTreeToFile(*tree, fileName);
}

inline PointCloudPtr LoadPointCloudFromFile(const std::string& filename) {
  PointCloudPtr cloud (new PointCloud);
  if (pcl::io::loadPCDFile<Point> (filename, *cloud) == -1) {
    throw std::runtime_error(std::string(__func__) + ": Can't read cloud file " + filename);
  }
  ROS_INFO_STREAM("Loaded " << cloud->size() << " data points from " << filename);
  return cloud;
}

inline void savePointCloudToFile(const std::string& filename, const PointCloud& cloud, bool binary) {
  if (pcl::io::savePCDFile<Point>(filename, cloud, binary) == -1) {
    throw std::runtime_error(std::string(__func__) + ": Can't save cloud to file " + filename);
  }
  ROS_INFO_STREAM("Saved " << cloud.size() << " data points to " << filename);
}

} // namespace octomap_tools