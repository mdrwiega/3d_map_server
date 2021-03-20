#include <octomap_tools/maps_integrator_visualizer.h>

#include <chrono>
#include <thread>

#include <ros/console.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>

#include <utils.h>
#include <conversions.h>

namespace octomap_tools {

void MapsIntegratorVisualizer::VisualizeICP(PointCloudPtr& scene, PointCloudPtr& model,
                                            const Eigen::Matrix4f& transformation) {
  // Transform model
  PointCloud::Ptr transformed_model(new PointCloud());
  pcl::transformPointCloud(*model, *transformed_model, transformation);

  Visualize(scene, model, nullptr, transformed_model, nullptr, nullptr, nullptr);
}

void MapsIntegratorVisualizer::VisualizeClouds(const PointCloudPtr& cloud1, const PointCloudPtr& cloud2) {
  Visualize(cloud1, cloud2);
}

void MapsIntegratorVisualizer::VisualizeFeatureMatching(const FeatureCloudPtr& scene, const FeatureCloudPtr& model,
                                                        const Eigen::Matrix4f& transformation,
                                                        const pcl::CorrespondencesPtr& correspondences) {
  auto scene_cloud = scene->GetPointCloud();
  auto model_cloud = model->GetPointCloud();
  auto scene_keypoints = model->GetKeypoints();
  auto model_keypoints = scene->GetKeypoints();

  // Transform model
  PointCloud::Ptr transformed_model(new PointCloud());
  pcl::transformPointCloud(*model_cloud, *transformed_model, transformation);

  Visualize(scene_cloud, model_cloud, nullptr, transformed_model, scene_keypoints, model_keypoints, correspondences);
}

void MapsIntegratorVisualizer::VisualizeFeatureMatchingWithDividedModel(
                                         PointCloudPtr& scene, PointCloudPtr& model,
                                         PointCloudPtr& full_model,
                                         const Eigen::Matrix4f& transformation,
                                         const std::vector<Rectangle>& blocks) {
  // Transform model
  PointCloud::Ptr transformed_model(new PointCloud());
  pcl::transformPointCloud(*model, *transformed_model, transformation);

  Visualize(scene, full_model, model, transformed_model, nullptr, nullptr, nullptr, blocks);
}

void MapsIntegratorVisualizer::Visualize(
  const PointCloudPtr& cloud1,
  const PointCloudPtr& cloud2,
  const PointCloudPtr& cloud3,
  const PointCloudPtr& cloud4,
  const PointCloudPtr& keypoints1,
  const PointCloudPtr& keypoints2,
  const pcl::CorrespondencesPtr& correspondences,
  const std::vector<Rectangle>& blocks) {

  pcl::visualization::PCLVisualizer viewer ("3D Viewer");
  viewer.setBackgroundColor(255, 255, 255);
  viewer.addCoordinateSystem(1.0);

  // Cloud1 / Scene --> BLUE
  if (cloud1) {
    pcl::visualization::PointCloudColorHandlerCustom<Point> color(cloud1, 0, 0, 255);
    viewer.addPointCloud(cloud1, color, "cloud1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
  }

  // Cloud2 / Model --> RED
  if (cloud2) {
    pcl::visualization::PointCloudColorHandlerCustom<Point> cloud2_color(cloud2, 255, 0, 0);
    viewer.addPointCloud(cloud2, cloud2_color, "cloud2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
  }

  // Cloud3 --> YELLOW
  if (cloud3) {
    pcl::visualization::PointCloudColorHandlerCustom<Point> color(cloud3, 255, 255, 0);
    viewer.addPointCloud(cloud3, color, "cloud3");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud3");
  }

  // Cloud4 --> GREEN
  if (cloud4) {
    pcl::visualization::PointCloudColorHandlerCustom<Point> color(cloud4, 0, 255, 0);
    viewer.addPointCloud(cloud4, color, "cloud4");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud4");
  }

  // Keypoints1 / Scene --> BLACK
  if (keypoints1) {
    pcl::visualization::PointCloudColorHandlerCustom<Point> color(keypoints1, 0, 0, 0);
    viewer.addPointCloud(keypoints1, color, "keypoints1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoints1");
  }

  // Keypoints2 / Model --> BLACK
  if (keypoints2) {
    pcl::visualization::PointCloudColorHandlerCustom<Point> color(keypoints2, 0, 0, 0);
    viewer.addPointCloud(keypoints2, color, "keypoints2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "keypoints2");
  }

  // Correspondences --> ORANGE
  if (correspondences) {
    for (size_t j = 0; j < (*correspondences).size (); ++j) {
      std::stringstream ss_line;
      ss_line << "correspondence_line" << j;
      Point& model_point = keypoints1->at((*correspondences)[j].index_query);
      Point& scene_point = keypoints2->at((*correspondences)[j].index_match);

      // Draw line for each pair of correspondences found between model and scene
      viewer.addLine<Point, Point>(model_point, scene_point, 1.0, 0.7, 0, ss_line.str());
    }
  }

  // Show grid
  int i = 0;
  for (auto& rec : blocks) {
    std::stringstream ss_line;
    ss_line << "rect" << i;
    viewer.addCube(rec.min(0), rec.max(0), rec.min(1), rec.max(1), 0, 0, 0, 0, 0.5, ss_line.str());
    ++i;
  }

  if (cfg_.save_to_file && !cfg_.filename.empty()) {
    viewer.spinOnce(100);
    viewer.saveScreenshot(cfg_.filename);
    ROS_INFO_STREAM("Screenshot has been saved to file: " << cfg_.filename << "\n");
  }

  if (!cfg_.screen_mode) {
    viewer.close();
  }

  while (!viewer.wasStopped()) {
    viewer.spinOnce(100);
  }
}

} // namespace octomap_tools
