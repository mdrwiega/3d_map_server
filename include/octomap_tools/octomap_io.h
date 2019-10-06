/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <memory>
#include <stdexcept>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <octomap/octomap.h>

#include <octomap_tools/utils.h>

namespace octomap_tools {

inline std::unique_ptr<OcTree> LoadOcTreeFromFile(const std::string& filename) {
  std::cout << "\nLoading octree from file: " << filename << std::endl;
  std::ifstream file(filename.c_str(), std::ios_base::in | std::ios_base::binary);
  if (!file.is_open()) {
    throw std::runtime_error(std::string(__func__) + ": Unable to open " + filename);
  }

  std::istream::pos_type streampos = file.tellg();
  std::unique_ptr<octomap::OcTree> tree;

  // Reading new format of octree (.ot)
  if (filename.length() > 3 && (filename.compare(filename.length()-3, 3, ".ot") == 0)) {
    tree.reset(dynamic_cast<octomap::OcTree*>(octomap::AbstractOcTree::read(file)));
    if (!tree) {
      file.clear();
      file.seekg(streampos);
      throw std::runtime_error(std::string(__func__) + ": Can't load octree from file " + filename);
    }
  } else { // Reading old format of octree (.bt)
    std::unique_ptr<octomap::OcTree> bTree (new octomap::OcTree(0.1));

    if (bTree->readBinary(file) && bTree->size() > 1) {
      tree.reset(bTree.release());
    } else {
      throw std::runtime_error(std::string(__func__) + ": Can't detect binary OcTree format in file " + filename);
    }
  }
  return tree;
}

inline void SaveOcTreeToFile(const OcTree& tree, const std::string& filename) {
  std::cout  << "\nSaving octree to file: " << filename << std::endl;
  if (!tree.write(filename)) {
    throw std::runtime_error(std::string(__func__) + ": Error during saving to " + filename);
  }
}

[[depracated("Lossy conversion from cloud to octree")]]
inline void SavePointCloudAsOctreeToFile(
    PointCloud::Ptr& cloud, const std::string& fileName, float resolution) {
  octomap::Pointcloud scan;

  for (const auto& i : cloud->points)
    scan.push_back(i.x, i.y, i.z);

  std::unique_ptr<octomap::OcTree> tree(new octomap::OcTree(resolution));

  octomap::point3d sensorPose{0,0,0};
  tree->insertPointCloud(scan, sensorPose);
  tree->updateInnerOccupancy();
  tree->prune();

  SaveOcTreeToFile(*tree, fileName);
}

inline PointCloudPtr LoadPointCloudFromFile(const std::string filename) {
  PointCloudPtr cloud (new PointCloud);
  if (pcl::io::loadPCDFile<Point> (filename, *cloud) == -1) {
    throw std::runtime_error(std::string(__func__) + ": Can't read cloud file " + filename);
  }
  std::cout << "Loaded " << cloud->size() << " data points from " << filename << std::endl;
  return cloud;
}

inline void savePointCloudToFile(const std::string filename, const PointCloud& cloud, bool binary) {
  if (pcl::io::savePCDFile<Point>(filename, cloud, binary) == -1) {
    throw std::runtime_error(std::string(__func__) + ": Can't save cloud to file " + filename);
  }
  std::cout << "Saved " << cloud.size() << " data points to " << filename << std::endl;
}

}
