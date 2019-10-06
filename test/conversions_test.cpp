/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <octomap_tools/types_conversions.h>

#include <gtest/gtest.h>

#include "test_utils.h"

using namespace octomap_tools;
using namespace Eigen;


TEST(OcTreeConversions, octree_to_cloud_to_octree) {
  auto tree_res = 0.1;
  octomap::OcTree tree(tree_res);

  // insert some measurements of occupied cells
  for (int x = -200; x < 200; x++) {
    for (int y = -200; y < 200; y++) {
      for (int z = -20; z < 20; z++) {
        octomap::point3d endpoint((float) x*0.05f, (float) y*0.05f, (float) z*0.05f);
        tree.updateNode(endpoint, true);
      }
    }
  }
  // insert some measurements of free cells
  for (int x = -30; x < 30; x++) {
    for (int y = -30; y < 30; y++) {
      for (int z = -30; z < 30; z++) {
        octomap::point3d endpoint((float) x*0.02f-1.0f, (float) y*0.02f-1.0f, (float) z*0.02f-1.0f);
        tree.updateNode(endpoint, false);
      }
    }
  }

  printOcTreeInfo(tree, "Tree before conversion");
  auto cloud = OcTreeToPointCloud(tree);
  auto tree2 = PointCloudToOctree(cloud, tree_res);
  printOcTreeInfo(tree2, "Tree after conversions");
  tree2.prune();
  printOcTreeInfo(tree2, "Tree after conversions");
}

