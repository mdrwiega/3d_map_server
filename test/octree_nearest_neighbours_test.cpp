/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "utils/OctreeUtils.hh"
#include "utils/Logger.hh"

#include <gtest/gtest.h>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "md_utils/math/transformations.hh"
#include "../src/octree_nearest_neighbours.h"

using namespace octomap_tools;
using namespace md_utils;
using namespace octomap;

TEST(OctreeNearestNeighboursTest, simple)
{
  constexpr float res = 0.1;

  OcTree tree(res);
  tree.createRoot();
  auto n1 = tree.createNodeChild(tree.getRoot(), 0);
  auto n2 = tree.createNodeChild(n1, 0);
  auto n3 = tree.createNodeChild(n2, 0);
  auto n4 = tree.createNodeChild(n3, 0);

  auto b3 = tree.createNodeChild(n2, 1);
  auto b4 = tree.createNodeChild(b3, 1);
  printOcTree(tree, "tree");

  double search_radius = 1000.0;
  Point search_point(-3000, -3072, -3072);

  // Brute force radius search
  std::vector<Point> bruteforce_search_points;
  for (auto leaf = tree.begin_leafs(); leaf != tree.end_leafs(); ++leaf)
  {
    point3d point = leaf.getCoordinate();
    Point pp(point.x()-search_point.x, point.y()-search_point.y, point.z()-search_point.z);

    auto pointDist = std::sqrt(squaredNorm(pp));
    if (pointDist <= search_radius)
      bruteforce_search_points.push_back(pp);
  }

  // Radius search
  std::vector<int> radius_search_points;
  std::vector<float> radius_search_dists;

  radiusSearch(tree, search_point, search_radius, radius_search_points, radius_search_dists, 1000);
  std::cout << " SearchRadius: " << search_radius << "\n";
  std::cout << " SEARCH BRUTEFORCE: " << bruteforce_search_points.size() << "\n";
  std::cout << " RADIUS SEARCH: " << radius_search_dists.size() << "\n";

  ASSERT_EQ (bruteforce_search_points.size(), radius_search_points.size());

  // heck if result from octree radius search can be also found in bruteforce search
  //    std::vector<int>::const_iterator current = cloudNWRSearch.begin ();
  //    while (current != radius_search_points.end ())
  //    {
  //      pointDist = sqrt (
  //          (cloudIn.points[*current].x - searchPoint.x) * (cloudIn.points[*current].x - searchPoint.x)
  //          + (cloudIn.points[*current].y - searchPoint.y) * (cloudIn.points[*current].y - searchPoint.y)
  //          + (cloudIn.points[*current].z - searchPoint.z) * (cloudIn.points[*current].z - searchPoint.z));
  //
  //      ASSERT_TRUE (pointDist <= searchRadius);
  //
  //      ++current;
  //    }
}

TEST(OctreeNearestNeighboursTest, Neighbours_Within_Radius_Search)
{

  const unsigned int test_runs = 1;
  unsigned int test_id;

  // instantiate point clouds
  PointCloud cloudIn;
  PointCloud cloudOut;

  srand (static_cast<unsigned int> (time (NULL)));

  for (test_id = 0; test_id < test_runs; test_id++)
  {
    // define a random search point
    Point searchPoint(static_cast<float> (10.0 * rand () / RAND_MAX),
                      static_cast<float> (10.0 * rand () / RAND_MAX),
                      static_cast<float> (10.0 * rand () / RAND_MAX));
    cloudIn.width = 1000;
    cloudIn.height = 1;
    cloudIn.points.resize (cloudIn.width * cloudIn.height);

    // generate point cloud data
    for (size_t i = 0; i < 1000; i++)
    {
      cloudIn.points[i] = Point(static_cast<float> (10.0 * rand () / (float)RAND_MAX),
                                static_cast<float> (10.0 * rand () / (float)RAND_MAX),
                                static_cast<float> (5.0  * rand () / (float)RAND_MAX));
    }

    OcTree tree(0.1);
    octomap::Pointcloud cloud;
    for (auto i : cloudIn)
      cloud.push_back(point3d{i.x,i.y,i.z});
    tree.insertPointCloud(cloud, point3d{0,0,0}, 100, false, false);

    std::cout << "Tree size= " << tree.size() << "\n";
    //    OctreePointCloudSearch<PointXYZ> octree (0.001);
    //
    //    // build octree
    //    octree.setInputCloud (cloudIn);
    //    octree.addPointsFromInputCloud ();
    //
    double pointDist;
    double searchRadius = 5.0;

    // bruteforce radius search
    std::vector<int> cloudSearchBruteforce;
    for (size_t i = 0; i < cloudIn.points.size (); i++)
    {
      Point pp(cloudIn.points[i].x-searchPoint.x, cloudIn.points[i].y-searchPoint.y, cloudIn.points[i].z-searchPoint.z);
      pointDist = std::sqrt (squaredNorm(pp));

      if (pointDist <= searchRadius)
      {
        // add point candidates to vector list
        cloudSearchBruteforce.push_back (static_cast<int> (i));
      }
    }

    std::cout << " CLOUD SEARCH BRUTEFORCE: " << cloudSearchBruteforce.size() << "\n";
    std::cout << " SearchRadius: " << searchRadius << "\n";

    std::vector<int> cloudNWRSearch;
    std::vector<float> cloudNWRRadius;

    // execute octree radius search//    octree.radiusSearch (searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius);
    radiusSearch(tree, searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius,1000);
    std::cout << " RADIUS SEARCH: " << cloudNWRRadius.size() << "\n";

    ASSERT_EQ (cloudSearchBruteforce.size (), cloudNWRRadius.size ());

    // check if result from octree radius search can be also found in bruteforce search
    std::vector<int>::const_iterator current = cloudNWRSearch.begin ();
    while (current != cloudNWRSearch.end ())
    {
      pointDist = sqrt (
          (cloudIn.points[*current].x - searchPoint.x) * (cloudIn.points[*current].x - searchPoint.x)
          + (cloudIn.points[*current].y - searchPoint.y) * (cloudIn.points[*current].y - searchPoint.y)
          + (cloudIn.points[*current].z - searchPoint.z) * (cloudIn.points[*current].z - searchPoint.z));

      ASSERT_TRUE (pointDist <= searchRadius);

      ++current;
    }

    // check if result limitation works
    radiusSearch(tree, searchPoint, searchRadius, cloudNWRSearch, cloudNWRRadius, 5);

    ASSERT_TRUE (cloudNWRRadius.size () <= 5);

  }

}
