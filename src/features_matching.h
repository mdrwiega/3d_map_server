/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#pragma once

#include <iostream>
#include <fstream>
#include <algorithm>

#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <pcl/point_types.h>

#include "utils/octree_utils.h"
#include <utils/pointcloud_utils.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>

#include <feature_cloud.h>
#include <utils/spiral_generator.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/registration/ia_ransac.h>

#include <template_alignment.h>

#include "utils/table_printer.h"

namespace octomap_tools {

class ICP {
 public:
  struct Config {
    unsigned max_iter{100};          // Max number of iterations (ICP)
    float    max_nn_dist{0.3};       // Max correspondence distance (NN)
    float    fitness_eps{0.05};       // Euclidean fitness epsilon (ICP)
    float    transf_eps{0.001};      // Transformation epsilon (ICP)
    float    scene_inflation_dist{1.0};
  };

};

class FeatureMatching {
  using NormalType = FeatureCloud::NormalType;
  using SurfaceNormals = FeatureCloud::SurfaceNormals;
  using DescriptorType = FeatureCloud::DescriptorType;
  using Descriptors = FeatureCloud::Descriptors;
  using RFType = FeatureCloud::RFType;

 public:
  struct Config {
    bool show_visualization_{false};
    bool show_keypoints_ {true};
    bool dump_to_file_{true};
    float model_ss_ {0.3f};
    float scene_ss_ {0.1f};
    float rf_rad_   {1.5};
    float descr_rad_ {1.5};
    float cg_size_ {0.5f};
    float cg_thresh_ {5.0f};
    unsigned model_size_thresh_{50};
    unsigned keypoints_thresh_{10};
    float cell_size_x_{2};
    float cell_size_y_{2};
    bool icp_correction{true};
    bool visualize_icp{false};
    std::string files_path_and_pattern;
    ICP::Config icp;
    FeatureCloud::Config feature_cloud;
    TemplateAlignment::Config template_alignment;

    std::string toString();
  };

  class Result {
   public:
    float time_ms;
    float fitness_score;
    Point model_min;
    Point model_max;
    Eigen::Matrix4f transformation;

    Result() :
      time_ms(0), fitness_score(0), model_min{}, model_max{}, transformation{} {}

      Result(float _time_ms, float _fitness_score, Point _model_min, Point _model_max, Eigen::Matrix4f _tf) {
        time_ms = _time_ms;
        fitness_score = _fitness_score;
        model_min = _model_min;
        model_max = _model_max;
        transformation = _tf;
      }

      std::string toString() {
        std::stringstream stream;
        md::TablePrinter tp(&stream);
        const std::string columns[] = {
            "time[ms]", "fitness_score", "model_min_x", "model_min_y", "model_max_x", "model_max_y"
        };

        for (const auto& i : columns) {
          tp.addColumn(i);
        }

        tp.printTitle("Feature matching results");
        tp.printHeader();

        tp << time_ms << fitness_score << model_min.x << model_min.y << model_max.x << model_max.y;

        tp.printFooter();
        return stream.str();
      }

      void PrintResult() {
        std::cout << "Result:" << std::endl;
        std::cout << "Processing time: " << time_ms / 1000.0 << " s." << std::endl;
        std::cout << "Fitness score: " << fitness_score << std::endl;
        std::cout << "Block  MIN: (" << model_min.x << ", " << model_min.y << ", " << model_min.z << ")  "
            << "MAX: (" << model_max.x << ", " << model_max.y << ", " << model_max.z << ")\n";
        PrintTransformationMatrix(transformation);
      }

      void PrintTransformationMatrix(Eigen::Matrix4f& Mat) {
        Eigen::Matrix3f rotation = Mat.block<3,3>(0, 0);
        Eigen::Vector3f translation = Mat.block<3,1>(0, 3);

        printf ("\n");
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
        printf ("        R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
        printf ("            | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
        printf ("\n");
        printf ("        t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
        printf ("\n");
      }
  };

  FeatureMatching(Config config) :
    template_align_({0.1, 5.0, 500}),
    cfg_(config) {
  }

  Result computeWithSingleModel(PointCloudPtr& scene_cloud, PointCloudPtr& model_cloud,
                                bool box_filter_model,
                                const Eigen::Vector4f& box_filter_min = Eigen::Vector4f(),
                                const Eigen::Vector4f& box_filter_max = Eigen::Vector4f());

  Result computeWithModelDivision(PointCloudPtr& cloud_l, PointCloudPtr& cloud_r);

  void setFullModel(PointCloudPtr& full_model);

  void DivideFullModelIntoBlocks(float block_size_x, float block_size_y);

  void DumpConfigAndResultsToFile();

  PointCloudPtr full_model_;
  std::vector<Rectangle> spiral_blocks_;
  TemplateAlignment template_align_;
  Config cfg_;
  Result result_;

};

}
