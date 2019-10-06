#include <octomap_tools/maps_integrator_visualizer.h>

#include <chrono>
#include <thread>

#include <octomap_tools/utils.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include "../include/octomap_tools/conversions.h"

namespace octomap_tools {

void MapsIntegratorVisualizer::visualize(const Eigen::Matrix4f& transformation,
                                         PointCloudPtr& scene, PointCloudPtr& model,
                                         PointCloudPtr& full_model,
                                         std::vector<Rectangle> blocks) {
  bool show_blocks = true;

  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor(255, 255, 255);
  viewer.addCoordinateSystem(1.0);
//  viewer.initCameraParameters();
//  viewer.setCameraPosition(0.0, 0.0, 25.0, 0.0, 0.0, 0.0);

  // Visualize scene point cloud --> BLUE
  pcl::visualization::PointCloudColorHandlerCustom<Point> scene_color(scene, 0, 0, 255);
  viewer.addPointCloud(scene, scene_color, "scene_cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene_cloud");

  // Visualize full model point cloud --> RED
  pcl::visualization::PointCloudColorHandlerCustom<Point> full_model_color(full_model, 255, 0, 0);
  viewer.addPointCloud(full_model, full_model_color, "full_model");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "full_model");

  // Visualize model before transformation --> RED
  pcl::visualization::PointCloudColorHandlerCustom<Point> model_color(model, 255, 255, 0);
  viewer.addPointCloud(model, model_color, "model");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model");

  // Visualize off scene model point cloud --> RED
  PointCloud::Ptr off_scene_model(new PointCloud());
  PointCloud::Ptr off_scene_model_keypoints(new PointCloud());

//  if (cfg_.show_keypoints) {
//    //  We are translating the model so that it doesn't end in the middle of the scene representation
//    pcl::transformPointCloud(*model, *off_scene_model,
//                             Eigen::Vector3f(-1, 0, 0),
//                             Eigen::Quaternionf(1, 0, 0, 0));
//    pcl::transformPointCloud(*model->getKeypoints(),
//                             *off_scene_model_keypoints,
//                             Eigen::Vector3f(-1, 0, 0),
//                             Eigen::Quaternionf(1, 0, 0, 0));
//    pcl::visualization::PointCloudColorHandlerCustom<Point> off_scene_model_color_handler(
//        off_scene_model, 255, 255, 128);
//    viewer.addPointCloud(off_scene_model, off_scene_model_color_handler,
//                         "off_scene_model");
//  }

//  if (cfg_.show_keypoints) {
//    pcl::visualization::PointCloudColorHandlerCustom<Point> scene_keypoints_color_handler(
//        scene->getKeypoints(), 0, 0, 255);
//    viewer.addPointCloud(scene->getKeypoints(), scene_keypoints_color_handler,
//                         "scene_keypoints");
//    viewer.setPointCloudRenderingProperties(
//        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");
//    pcl::visualization::PointCloudColorHandlerCustom<Point> off_scene_model_keypoints_color_handler(
//        off_scene_model_keypoints, 0, 0, 255);
//    viewer.addPointCloud(off_scene_model_keypoints,
//                         off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
//  }

  PointCloud::Ptr rotated_model(new PointCloud());
  pcl::transformPointCloud(*model, *rotated_model, transformation);
  std::stringstream ss_cloud;
  ss_cloud << "instance1";
  pcl::visualization::PointCloudColorHandlerCustom<Point> rotated_model_color_handler(rotated_model, 0, 255, 0);
  viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, ss_cloud.str());

  if (show_blocks) {
    int i = 0;
    for (auto& rec : blocks) {
      std::stringstream ss_line;
      ss_line << "rect" << i;
      viewer.addCube(rec.min(0), rec.max(0), rec.min(1), rec.max(1), 0, 0, 0, 0, 0.5, ss_line.str());
      ++i;
    }
  }

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (cfg_.save_to_file && !cfg_.filename.empty()) {
    viewer.saveScreenshot(cfg_.filename);
    std::cout << "Screenshot has been saved to file: " << cfg_.filename << "\n";
  }
}

void MapsIntegratorVisualizer::visualizeICP(PointCloudPtr& scene, PointCloudPtr& model,
                                             const Eigen::Matrix4f& transformation) {
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor(255, 255, 255);
  viewer.addCoordinateSystem(1.0);

  // Scene --> BLUE
  pcl::visualization::PointCloudColorHandlerCustom<Point> scene_color(scene, 0, 0, 255);
  viewer.addPointCloud(scene, scene_color, "scene");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "scene");

  // Model before transformation --> RED
  pcl::visualization::PointCloudColorHandlerCustom<Point> full_model_color(model, 255, 0, 0);
  viewer.addPointCloud(model, full_model_color, "model");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "model");

  // Model after transformation --> GREEN
  PointCloud::Ptr rotated_model(new PointCloud());
  pcl::transformPointCloud(*model, *rotated_model, transformation);
  pcl::visualization::PointCloudColorHandlerCustom<Point> rotated_model_color_handler(rotated_model, 0, 255, 0);
  viewer.addPointCloud(rotated_model, rotated_model_color_handler, "transformed_model");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "transformed_model");

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (cfg_.save_to_file && !cfg_.filename.empty()) {
    viewer.saveScreenshot(cfg_.filename);
    std::cout << "Screenshot has been saved to file: " << cfg_.filename << "\n";
  }
}

void MapsIntegratorVisualizer::visualizeClouds(const PointCloudPtr& cloud1, const PointCloudPtr& cloud2) {
  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor(255, 255, 255);
  viewer.addCoordinateSystem(1.0);

  // Scene --> BLUE
  pcl::visualization::PointCloudColorHandlerCustom<Point> scene_color(cloud1, 0, 0, 255);
  viewer.addPointCloud(cloud1, scene_color, "cloud1");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");

  // Model before transformation --> RED
  pcl::visualization::PointCloudColorHandlerCustom<Point> full_model_color(cloud2, 255, 0, 0);
  viewer.addPointCloud(cloud2, full_model_color, "cloud2");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");

  while (!viewer.wasStopped())
  {
    viewer.spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  if (cfg_.save_to_file && !cfg_.filename.empty()) {
    viewer.saveScreenshot(cfg_.filename);
    std::cout << "Screenshot has been saved to file: " << cfg_.filename << "\n";
  }
}

}
