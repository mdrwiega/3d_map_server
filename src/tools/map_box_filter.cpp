#include <cstdlib>
#include <iostream>

#include <Eigen/Dense>

#include <octomap_tools/utils.h>
#include <octomap_tools/octomap_io.h>

#include <octomap_tools/transformations.h>

using namespace octomap_tools;

void printHelp(const char* progName) {
  std::cout << "Usage: " << progName
            << " <in_file.ot> <out_file1.ot>"
            << " x_min x_max y_min y_max z_min z_max\n"
            << "This program filters octree.\n";
  exit(0);
}

int main(int argc, char** argv) {
  if (argc != 9) {
    printHelp(argv[0]);
  }

  const std::string in_filepath = argv[1];
  const std::string out_filepath = argv[2];

  const float x_min = std::strtof(argv[3], nullptr);
  const float x_max = std::strtof(argv[4], nullptr);
  const float y_min = std::strtof(argv[5], nullptr);
  const float y_max = std::strtof(argv[6], nullptr);
  const float z_min = std::strtof(argv[7], nullptr);
  const float z_max = std::strtof(argv[8], nullptr);

  auto min = Eigen::Vector3f(x_min, y_min, z_min);
  auto max = Eigen::Vector3f(x_max, y_max, z_max);

  auto in_tree = LoadOcTreeFromFile(in_filepath);
  auto out_tree = CropOcTree(*in_tree, min, max);

  PrintOcTreeInfo(*in_tree, "Input tree");
  PrintOcTreeInfo(*out_tree, "Output tree");

  SaveOcTreeToFile(*out_tree, out_filepath);

  return 0;
}