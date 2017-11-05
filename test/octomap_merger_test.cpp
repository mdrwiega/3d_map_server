/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <gtest/gtest.h>

#include "octomap_merger.h"

#include "utils/octree_utils.h"
#include "utils/logger.h"
#include "octree_transformations.h"
#include "octree_icp.h"
#include "test_utils.h"

using namespace octomap_tools;
using namespace octomap;

TEST(OctomapMergerTest, mergeOcTrees_NodesInTheSameDepth)
{
  constexpr float res = 0.1;

  OcTree tree1(res);
  tree1.createRoot();
  auto n = tree1.createNodeChild(tree1.getRoot(), 0);
  n = tree1.createNodeChild(n, 0);
  n = tree1.createNodeChild(n, 0);
  n->setLogOdds(logodds(0.6));

  OcTree tree2(res);
  tree2.createRoot();
  n = tree2.createNodeChild(tree2.getRoot(), 0);
  n = tree2.createNodeChild(n, 0);
  n = tree2.createNodeChild(n, 0);
  n->setLogOdds(logodds(0.6));

  auto treeOut = sumOctrees(tree1, tree2);

  printOcTree(tree1, "Tree1");
  printOcTree(tree2, "Tree2");
  printOcTree(*treeOut, "Out tree");

  EXPECT_EQ(1U, treeOut->getNumLeafNodes());
}

TEST(OctomapMergerTest, mergeOcTrees_NodeInTheTree2IsDeeper)
{
  constexpr float res = 0.1;

  OcTree tree1(res);
  tree1.createRoot();
  auto n = tree1.createNodeChild(tree1.getRoot(), 0);
  n = tree1.createNodeChild(n, 0);
  n->setLogOdds(logodds(0.4));

  OcTree tree2(res);
  tree2.createRoot();
  n = tree2.createNodeChild(tree2.getRoot(), 0);
  n = tree2.createNodeChild(n, 0);
  n = tree2.createNodeChild(n, 0);
  n->setLogOdds(logodds(0.6));

  auto treeOut = sumOctrees(tree1, tree2);

  printOcTree(tree1, "Tree1");
  printOcTree(tree2, "Tree2");
  printOcTree(*treeOut, "Out tree");

  EXPECT_EQ(8U, treeOut->getNumLeafNodes());
}

TEST(OctomapMergerTest, mergeOcTrees_NodeInTheTree1IsDeeper)
{
  constexpr float res = 0.1;

  OcTree tree1(res);
  tree1.createRoot();
  auto n = tree1.createNodeChild(tree1.getRoot(), 0);
  n = tree1.createNodeChild(n, 0);
  n = tree1.createNodeChild(n, 0);
  n->setLogOdds(logodds(0.3));

  OcTree tree2(res);
  tree2.createRoot();
  n = tree2.createNodeChild(tree2.getRoot(), 0);
  n = tree2.createNodeChild(n, 0);
  n->setLogOdds(logodds(0.7));

  printOcTree(tree1, "Tree1");
  printOcTree(tree2, "Tree2");

  auto treeOut = sumOctrees(tree1, tree2);

  printOcTree(*treeOut, "Out tree");

  EXPECT_EQ(8U, treeOut->getNumLeafNodes());
}

