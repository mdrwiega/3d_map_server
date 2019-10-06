/******************************************************************************
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <octomap_tools/conversions.h>

#include <gtest/gtest.h>

#include "test_utils.h"

using namespace octomap_tools;
using namespace Eigen;

TEST(OcTreeConversions, octree_to_cloud) {
  auto tree_res = 0.1;
  octomap::OcTree tree(tree_res);

  // insert some measurements of occupied cells
  for (int x = -200; x < 200; x++) {
    for (int y = -20; y < 20; y++) {
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

  PrintOcTreeInfo(tree, "Tree before conversion");
  auto cloud = OcTreeToPointCloud(tree);

  // Calculate num of occupied nodes in tree for comparison
  tree.expand();
  unsigned num_occ_leafs = 0;
  for (auto i = tree.begin_leafs(); i != tree.end_leafs(); ++i) {
    if (tree.isNodeOccupied(*i)) {
      num_occ_leafs++;
    }
  }

  EXPECT_EQ(cloud.size(), num_occ_leafs);
}

