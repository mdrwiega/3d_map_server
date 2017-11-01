/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <iomanip>
#include <stdexcept>
#include <algorithm>

#include "md_utils/math/transformations.hh"

#include "octomap_merger.h"
#include "octree_transformations.h"

using namespace octomap;
using namespace Eigen;
using namespace md_utils;

namespace octomap_tools {

OcTreePtr OctomapMerger::sumOctrees(OcTree& tree1, OcTree& tree2)
{
  OcTreePtr tree_out = std::make_unique<OcTree>(tree1);

  for (auto leaf2 = tree2.begin_leafs(); leaf2 != tree2.end_leafs(); ++leaf2)
  {
    point3d point = leaf2.getCoordinate();
    OcTreeNode* leaf1 = tree_out->search(point);

    // Node in tree1 not exists. Just simply add it.
    if (leaf1 == nullptr)
    {
      auto new_node = tree_out->updateNode(point, true);
      new_node->setLogOdds(leaf2->getLogOdds());
    }
    else
    {
      int depth1 = getLeafDepth(*tree_out, *leaf1);
      if (depth1 != -1)
      {
        int depth2 = leaf2.getDepth();
        int depth_diff = depth2 - depth1;
        auto leaf2_logodds = leaf2->getLogOdds();

        // Nodes at the same level
        if (depth_diff == 0)
        {
          tree_out->updateNodeLogOdds(leaf1, leaf2_logodds);
        }
        // Node in tree2 is on deeper level than in tree1
        else if (depth_diff > 0)
        {
          for(int i = 0; i < depth_diff; i++)
          {
            tree_out->expandNode(leaf1);
            leaf1 = tree_out->search(point);
          }
          tree_out->updateNodeLogOdds(leaf1, leaf2_logodds);
        }
        // Node in tree1 is on deeper level than in tree2
        else if (depth_diff < 0)
        {
          for (int i = depth2; i < depth1; i++)
          {
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

}
