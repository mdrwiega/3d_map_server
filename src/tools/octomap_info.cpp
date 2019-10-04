/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, Michal Drwiega (drwiega.michal@gmail.com)
 * All rights reserved.
 *****************************************************************************/

#include <cstdlib>
#include <iostream>

#include <octomap_tools/utils.h>

using namespace octomap_tools;
using namespace Eigen;

void printHelp(const char* progName) {
  std::cout << "Usage: " << progName
      << " <in_file.ot>\n"
      << "This program shows octomap info.\n";
  exit(0);
}

int main(int argc, char** argv) {
  if (argc != 2) {
    printHelp(argv[0]);
  }

  const std::string inFilename = argv[1];
  auto tree = loadOctreeFromFile(inFilename);
  printOcTreeInfo(*tree, "Octomap info");
  return 0;
}
