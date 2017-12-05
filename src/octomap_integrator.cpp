#include "octomap_integrator.h"

#include <thread>

#include "octomap_merger.h"
#include "octree_transformations.h"
#include "octree_icp.h"
#include "md_utils/math/transformations.h"
#include "utils/types_conversions.h"
#include <pcl/visualization/pcl_visualizer.h>

#define SHOW_PCL 0

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

Eigen::Matrix4f estimateTransBetweenOctomapsPcl(
    const OcTree& tree1, const OcTree& tree2,
    const OctreeIntegrationConf& conf)
{
  auto cloud1 = octreeToPointCloud(tree1);
  auto cloud2 = octreeToPointCloud(tree2);
  return estimateTransBetweenPointclouds(cloud1, cloud2, conf);
}

Eigen::Matrix4f estimateTransBetweenPointclouds(
    const PointCloud& cloud1, const PointCloud& cloud2,
    const OctreeIntegrationConf& params)
{
  PointCloud::Ptr source(new PointCloud);
  PointCloud::Ptr target(new PointCloud);

  extractIntersectingAndDownsamplePointClouds(
      cloud1, cloud2, params.voxel_size, params.intersec_margin, *source, *target);

#if SHOW_PCL == 1
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor (0, 0, 0);
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters();
  viewer.setCameraPosition(0.0, 0.0, 25.0, 0.0, 0.0, 0.0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(source, 0, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ> (source, single_color, "source");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2(target, 255, 0, 255);
  viewer.addPointCloud<pcl::PointXYZ> (target, single_color2, "target");

  Point min1, max1, min2, max2;
  pcl::getMinMax3D(*source, min1, max1);
  pcl::getMinMax3D(*target, min2, max2);

  viewer.addCube(min1.x, max1.x, min1.y, max1.y, min1.z, max1.z, 0, 0, 1, "source");
  viewer.addCube(min2.x, max2.x, min2.y, max2.y, min2.z, max2.z, 1, 0, 1, "target");
#endif

  pcl::IterativeClosestPoint <Point, Point> icp;
  icp.setMaxCorrespondenceDistance(params.max_nn_dist);
  icp.setMaximumIterations(params.max_iter);
  icp.setTransformationEpsilon(params.transf_eps);
  icp.setEuclideanFitnessEpsilon (params.fitness_eps);

  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.align(*source);
  std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
  std::cout << "ICP transformation " << " : cloud_icp -> cloud_in" << std::endl;

#if SHOW_PCL == 1
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color3(source, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (source, single_color3, "src");
  pcl::getMinMax3D(*source, min1, max1);
  viewer.addCube(min1.x, max1.x, min1.y, max1.y, min1.z, max1.z, 0, 1, 0, "source2");

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
#endif

  return icp.getFinalTransformation();
}

}
