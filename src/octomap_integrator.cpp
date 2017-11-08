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
  T_fin = estimateTransBetweenOctomaps(tree1, tree2, conf);
  LOG_DBG() << "ICP has been finished\nEstimated rotation: "
      << T_fin.block<3,3>(0,0).eulerAngles(0, 1, 2).transpose()
      << "\nEstimated translation: "
      << T_fin.block<3,1>(0,3).transpose() << "\n";

  auto trans_src_tree = transformOctree(tree1, T_fin);
  LOG_DBG() << "Octree has been transformed\n";

  auto merged_tree = sumOctrees(*trans_src_tree, tree2);
  LOG_DBG() << "Octrees have been merged\n";
  return merged_tree;
}

Eigen::Matrix4f estimateTransBetweenOctomaps(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf)
{
  OcTree src_tree_f(1);
  OcTree dst_tree_f(1);

  extractIntersectingOctrees(
      tree1, tree2, conf.intersec_margin, src_tree_f, dst_tree_f);

  return icp(src_tree_f, dst_tree_f, conf.max_iter, conf.fitness_eps);
}


OcTreePtr integrateOctomapsPcl(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf,
    const Eigen::Matrix4f& T_init, Eigen::Matrix4f& T_fin,
    float& error)
{
  PointCloud cloud_src = octreeToPointCloud(tree1);
  PointCloud cloud_dst = octreeToPointCloud(tree2);

  T_fin = estimateTransBetweenPointclouds(cloud_src, cloud_dst, conf);
  LOG_DBG() << "ICP has been finished\nEstimated rotation: "
      << T_fin.block<3,3>(0,0).eulerAngles(0, 1, 2).transpose()
      << "\nEstimated translation: "
      << T_fin.block<3,1>(0,3).transpose() << "\n";

  auto trans_src_tree = transformOctree(tree1, T_fin);
  LOG_DBG() << "Octree has been transformed\n";

  auto merged_tree = sumOctrees(*trans_src_tree, tree2);
  LOG_DBG() << "Octrees have been merged\n";
  return merged_tree;
}

Eigen::Matrix4f estimateTransBetweenPointclouds(
    const PointCloud& cloud1, const PointCloud& cloud2,
    const OctreeIntegrationConf& params)
{
  PointCloud::Ptr source(new PointCloud);
  PointCloud::Ptr target(new PointCloud);

  extractIntersectingAndDownsamplePointClouds(
      cloud1, cloud2, params.voxel_size, params.intersec_margin, *source, *target);

  pcl::IterativeClosestPoint <Point, Point> icp;
  icp.setMaxCorrespondenceDistance(params.max_nn_dist);
  icp.setMaximumIterations(params.max_iter);
  icp.setTransformationEpsilon(params.transf_eps);
  icp.setEuclideanFitnessEpsilon (params.fitness_eps);

  PointCloud::Ptr icpResult;

  icp.setInputSource(source);
  icp.setInputTarget(target);
  icpResult = source;
  icp.align(*icpResult);

  return icp.getFinalTransformation();
}

}
