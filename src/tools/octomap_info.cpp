#include <cstdlib>
#include <iostream>

#include <octomap_tools/utils.h>
#include <octomap_tools/octomap_io.h>

using namespace octomap_tools;

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

  const std::string in_filename = argv[1];
  auto tree = LoadOcTreeFromFile(in_filename);
  PrintOcTreeInfo(*tree, "Octomap info");
  return 0;
}
