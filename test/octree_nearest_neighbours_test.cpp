/******************************************************************************
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include "utils/OctreeUtils.hh"
#include "utils/Logger.hh"

#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "md_utils/math/transformations.hh"
#include "octree_nearest_neighbours.h"

using namespace octomap_tools;
using namespace md_utils;
using namespace octomap;

void BruteForceRadiusSearch(const OcTree& tree,
                            const Point& search_point, float search_radius,
                            std::vector<Point>& points,
                            std::vector<float>& distances)
{
  float squared_radius = search_radius * search_radius;

  for (auto leaf = tree.begin_leafs(); leaf != tree.end_leafs(); ++leaf)
  {
    point3d p = leaf.getCoordinate();

    Point pp(p.x() - search_point.x, p.y() - search_point.y, p.z() - search_point.z);
    auto squared_point_dist = squaredNorm(pp);
    if (squared_point_dist <= squared_radius)
    {
      points.push_back(Point(p.x(), p.y(), p.z()));
      distances.push_back(std::sqrt(squared_point_dist));
    }
  }
}

void BruteForceNearestNeighbourSearch(const OcTree& tree, const Point& query,
                                      double max_distance, Point& point, float& distance)
{
  std::vector<Point> points;
  std::vector<float> distances;
  BruteForceRadiusSearch(tree, query, max_distance, points, distances);
//  printPointsAndDistances("BF", points, distances);

  auto min_index = std::min_element(distances.begin(), distances.end()) - distances.begin();
  point = points[min_index];
  distance = distances[min_index];
}

pcl::PointCloud<pcl::PointXYZ> GenerateRandomPointcloud(unsigned size)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = size;
  cloud.height = 1;
  cloud.points.resize (cloud.width * cloud.height);

  for (size_t i = 0; i < size; ++i)
  {
    cloud.points[i] = Point(static_cast<float>(15.0 * std::rand() / (float)RAND_MAX),
                            static_cast<float>(5.0  * std::rand() / (float)RAND_MAX),
                            static_cast<float>(10.0 * std::rand() / (float)RAND_MAX));
  }
  return cloud;
}

OcTree ConvertPointCloudToOctree(const PointCloud& cloud, double tree_resolution)
{
  OcTree tree(tree_resolution);

  octomap::Pointcloud cloud_octo;
  for (auto i : cloud)
    cloud_octo.push_back(point3d{i.x, i.y, i.z});
  tree.insertPointCloud(cloud_octo, point3d{0,0,0}, 100, false, false);

  return tree;
}

bool checkIfResultsFromOneMethodContainsResultsFromSecond(
    const std::vector<Point>& vec1, const std::vector<Point> vec2)
{
  for (auto i : vec2)
  {
    auto el = std::find_if(vec1.begin(), vec1.end(), [&i](auto& e) {
      return i.x == e.x && i.y == e.y && i.z == e.z; });

    if (el == vec1.end())
      return false;
  }
  return true;
}

TEST(OctreeNearestNeighboursTest, NeighboursWithinRadiusSearch_SimpleTree_TwoBranches)
{
  OcTree tree(0.1);
  tree.createRoot();
  auto n1 = tree.createNodeChild(tree.getRoot(), 0);
  auto n2 = tree.createNodeChild(n1, 0);
  auto n3 = tree.createNodeChild(n2, 0);
  tree.createNodeChild(n3, 0);

  auto b3 = tree.createNodeChild(n2, 1);
  tree.createNodeChild(b3, 1);
  printOcTree(tree, "tree");

  double search_radius = 1000.0;
  Point search_point(-3000, -3070, -3070);

  // Brute force radius search
  std::vector<Point> bruteforce_search_points;
  std::vector<float> bruteforce_search_distances;
  BruteForceRadiusSearch(tree, search_point, search_radius,
                         bruteforce_search_points, bruteforce_search_distances);

  // Radius search
  std::vector<Point> radius_search_points;
  std::vector<float> radius_search_dists;
  radiusSearch(tree, search_point, search_radius,
               radius_search_points, radius_search_dists);

  std::cout << " Search Radius: " << search_radius << "\n";
  std::cout << " Search Bruteforce: " << bruteforce_search_points.size() << "\n";
  std::cout << " Radius Search: " << radius_search_dists.size() << "\n";

  ASSERT_EQ(bruteforce_search_points.size(), radius_search_points.size());

  // Check if results from radius search are available in brute force results
  for (auto i : radius_search_points)
  {
    auto el = std::find_if(bruteforce_search_points.begin(),
                           bruteforce_search_points.end(),
                           [&i](auto&e){ return (i.x == e.x && i.y == e.y && i.z == e.z); });
    ASSERT_TRUE(el == bruteforce_search_points.end());
  }
}

TEST(OctreeNearestNeighboursTest, NeighboursWithinRadiusSearch)
{
  constexpr std::size_t test_runs = 1;
  constexpr unsigned points_size = 100;
  std::srand(std::time(0));

  for (std::size_t test_id = 0; test_id < test_runs; test_id++)
  {
    // Generate random pointcloud and convert it to octree
    PointCloud cloud = GenerateRandomPointcloud(points_size);
    OcTree tree = ConvertPointCloudToOctree(cloud, 0.1);

    double search_radius = 5.0;

    // Random search point
    Point search_point(static_cast<float>(5.0 * std::rand() / (float)RAND_MAX),
                       static_cast<float>(5.0 * std::rand() / (float)RAND_MAX),
                       static_cast<float>(5.0 * std::rand() / (float)RAND_MAX));

    // Brute force radius search
    std::vector<Point> bruteforce_search_points;
    std::vector<float> bruteforce_search_distances;
    BruteForceRadiusSearch(tree, search_point, search_radius,
                           bruteforce_search_points, bruteforce_search_distances);

    // Radius search
    std::vector<Point> radius_search_points;
    std::vector<float> radius_search_distances;
    radiusSearch(tree, search_point, search_radius,
                 radius_search_points, radius_search_distances);

    ASSERT_EQ(bruteforce_search_points.size(), radius_search_points.size());

    // Check if results from radius search are available in brute force results
    for (auto i : radius_search_points)
    {
      auto el = std::find_if(bruteforce_search_points.begin(),
                             bruteforce_search_points.end(),
                             [&i](auto&e){ return (i.x == e.x && i.y == e.y && i.z == e.z); });
      ASSERT_TRUE(el == bruteforce_search_points.end());
    }
  }
}

TEST(OctreeNearestNeighboursTest, NearestNeighbourSearch_SimpleTree_TwoBranches)
{
  OcTree tree(0.1);
  tree.createRoot();
  auto n1 = tree.createNodeChild(tree.getRoot(), 0);
  auto n2 = tree.createNodeChild(n1, 0);
  auto n3 = tree.createNodeChild(n2, 0);
  tree.createNodeChild(n3, 0);

  auto b3 = tree.createNodeChild(n2, 1);
  tree.createNodeChild(b3, 1);
  printOcTree(tree, "tree");

  double max_dist = 1000.0;
  Point search_point(-3000, -3070, -3070);

  // Brute force radius search
  Point bruteforce_point;
  float bruteforce_dist;
  BruteForceNearestNeighbourSearch(
      tree, search_point, max_dist, bruteforce_point, bruteforce_dist);

  // Brute force radius search
  Point nearest_neighbour_point;
  float nearest_neighbour_dist;
  searchNearestNeighbour(
      tree, search_point, max_dist, nearest_neighbour_point, nearest_neighbour_dist);

  ASSERT_FLOAT_EQ(bruteforce_dist, nearest_neighbour_dist);
  ASSERT_EQ(bruteforce_point.x, nearest_neighbour_point.x);
  ASSERT_EQ(bruteforce_point.y, nearest_neighbour_point.y);
  ASSERT_EQ(bruteforce_point.z, nearest_neighbour_point.z);
}

TEST(OctreeNearestNeighboursTest, NearestNeighbourSearch_Multi_Test)
{
  constexpr std::size_t test_runs = 1;
  constexpr unsigned points_size = 100;
  std::srand(std::time(0));

  for (std::size_t test_id = 0; test_id < test_runs; test_id++)
  {
    // Generate random pointcloud and convert it to octree
    PointCloud cloud = GenerateRandomPointcloud(points_size);
    OcTree tree = ConvertPointCloudToOctree(cloud, 0.1);

    // Random search point
    Point search_point(static_cast<float>(5.0 * std::rand() / (float)RAND_MAX),
                       static_cast<float>(5.0 * std::rand() / (float)RAND_MAX),
                       static_cast<float>(5.0 * std::rand() / (float)RAND_MAX));
    double max_dist = 5.0;

    // Brute force radius search
    Point bruteforce_point;
    float bruteforce_dist;
    BruteForceNearestNeighbourSearch(
        tree, search_point, max_dist, bruteforce_point, bruteforce_dist);

    // Brute force radius search
    Point nearest_neighbour_point;
    float nearest_neighbour_dist;
    searchNearestNeighbour(
        tree, search_point, max_dist, nearest_neighbour_point, nearest_neighbour_dist);

    ASSERT_FLOAT_EQ(bruteforce_dist, nearest_neighbour_dist);
    ASSERT_EQ(bruteforce_point.x, nearest_neighbour_point.x);
    ASSERT_EQ(bruteforce_point.y, nearest_neighbour_point.y);
    ASSERT_EQ(bruteforce_point.z, nearest_neighbour_point.z);
  }
}
