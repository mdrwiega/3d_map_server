#include "octomap_integrator.h"

#include "octomap_merger.h"
#include "octree_transformations.h"
#include "octree_icp.h"
#include "md_utils/math/transformations.h"
#include "utils/types_conversions.h"

namespace octomap_tools {

OcTreePtr integrateOctomaps(const OcTree& tree1, const OcTree& tree2,
                            const OctreeIntegrationConf& conf,
                            const Eigen::Matrix4f& T_init,
                            Eigen::Matrix4f& T_fin,
                            float& error)
{
  OcTree src_tree_f(1);
  OcTree dst_tree_f(1);

  extractIntersectingOctrees(
      tree1, tree2, conf.intersec_margin, src_tree_f, dst_tree_f);

  printOcTreeInfo(src_tree_f, "src_tree_filtered");
  printOcTreeInfo(dst_tree_f, "dst_tree_filtered");

  Eigen::Matrix3f R_est;
  Eigen::Vector3f T_est;
  error = icp(src_tree_f, dst_tree_f, R_est, T_est,
              conf.max_iter, conf.fitness_eps);

  LOG_DBG() << "ICP has been finished\n";

  T_fin = md::transformationMat(R_est, T_est);
  auto trans_src_tree = transformOctree(tree1, T_fin);
  LOG_DBG() << "Octree has been transformed\n";

  auto merged_tree = sumOctrees(*trans_src_tree, tree2);
  LOG_DBG() << "Octrees have been merged\n";
  return merged_tree;
}

OcTreePtr integrateOctomapsPcl(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf,
    const Eigen::Matrix4f& T_init, Eigen::Matrix4f& T_fin,
    float& error)
{
  PointCloud cloud_src;
  OctreeToPointCloud(&tree1, cloud_src);
  PointCloud cloud_dst;
  OctreeToPointCloud(&tree2, cloud_dst);

  EstimationParams params;
  params.maxIter = conf.max_iter;
  params.maxCorrespondenceDist = conf.max_nn_dist;
  params.fitnessEps = conf.fitness_eps;
  params.transfEps = 0.01;
  params.voxelSize = 0.1;
  params.intersecMargin = conf.intersec_margin;

  T_fin = computeTransBetweenPointclouds(cloud_src, cloud_dst, params);

  auto trans_src_tree = transformOctree(tree1, T_fin);
  LOG_DBG() << "Octree has been transformed\n";

  auto merged_tree = sumOctrees(*trans_src_tree, tree2);
  LOG_DBG() << "Octrees have been merged\n";
  return merged_tree;
}

}
