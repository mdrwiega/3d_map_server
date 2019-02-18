cmake_minimum_required(VERSION 2.8.2)

find_package(Threads REQUIRED)

# Download and install GoogleTest
include(ExternalProject)
ExternalProject_Add(googletest
  GIT_REPOSITORY    https://github.com/google/googletest.git
  GIT_TAG           master
  SOURCE_DIR        "${CMAKE_CURRENT_BINARY_DIR}/googletest-src"
  BINARY_DIR        "${CMAKE_CURRENT_BINARY_DIR}/googletest-build"
  CONFIGURE_COMMAND ""
  BUILD_COMMAND     ""
  INSTALL_COMMAND   ""
  TEST_COMMAND      ""
)
 
set(SRC_DIR "../src")

add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

include_directories(
    ${SRC_DIR}
)

enable_testing()

set(TESTED_SRC
    ${SRC_DIR}/octomap_merger.cpp
    ${SRC_DIR}/octomap_integrator.cpp
    ${SRC_DIR}/utils/octree_utils.cpp
    ${SRC_DIR}/octree_nearest_neighbours.cpp
    ${SRC_DIR}/octree_transformations.cpp
    ${SRC_DIR}/pointcloud_transformations.cpp
    ${SRC_DIR}/octree_icp.cpp
    ${SRC_DIR}/kdtree/kdtree.c
)

file(GLOB TESTS_SRC
	"main.cpp"
#	"*.cpp"
   "correspondence_grouping_test.cpp"
#	"octomap_integrator_test.cpp"
)

add_executable(octomapToolsTests ${TESTS_SRC} ${TESTED_SRC})

target_link_libraries(octomapToolsTests gtest_main mdUtils)
add_test(NAME octomapToolsTests COMMAND octomapToolsTests)
