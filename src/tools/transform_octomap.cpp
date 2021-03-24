#include <cstdlib>
#include <iostream>

#include <Eigen/Dense>

#include <utils.h>
#include <octomap_tools/octomap_io.h>

#include <transformations.h>
#include <conversions.h>

using namespace octomap_tools;

// Transform octomap

void printHelp(const char* progName) {
  std::cout << "Usage: " << progName
            << " <in_file.ot> <out_file1.ot>"
            << " x y z roll pitch yaw\n"
            << "This program filters octree.\n";
  exit(0);
}

int main(int argc, char** argv) {
  if (argc != 9) {
    printHelp(argv[0]);
  }

  const std::string in_filepath = argv[1];
  const std::string out_filepath = argv[2];

  const float x = std::strtof(argv[3], nullptr);
  const float y = std::strtof(argv[4], nullptr);
  const float z = std::strtof(argv[5], nullptr);
  const float r = std::strtof(argv[6], nullptr);
  const float p = std::strtof(argv[7], nullptr);
  const float yaw = std::strtof(argv[8], nullptr);

  auto in_tree = LoadOcTreeFromFile(in_filepath);

  auto T = createTransformationMatrix(x, y, z, ToRad(r), ToRad(p), ToRad(yaw));
  auto out_tree = FastOcTreeTransform(*in_tree, T);

  PrintOcTreeInfo(*in_tree, "Input tree");
  PrintOcTreeInfo(*out_tree, "Output tree");

  SaveOcTreeToFile(*out_tree, out_filepath);

  return 0;
}