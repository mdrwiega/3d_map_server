# 3d_map_server

The package contains the 3D map server.
Currently it supports ROS1 but integration with ROS2 is planned as well.

Main features of the 3D map server:
- it can merge maps based on the overlapping region

![Feature base matching example](design/matching_example.png)

## Status

It's still on an early stage of the development.

## Design

![Class Diagram](design/class_diagram.png)
Not fully implemented design.

## Compilation

- Clone repository to the ROS workspace (e.g. ros_ws/src)
  `git clone https://gitlab.com/mdrwiega/3d_map_server.git`
- Compile the projects
  `catkin_make`

## Tests

- Compile tests.
  `catkin_make tests`

- Compile and run all tests
  `catkin_make run_tests 3d_map_server`

- Run merge octomaps test - fr079
  `~/ros_ws/devel/lib/3d_map_server/3d_map_server_test --gtest_filter=MapsIntegratorTest.Test_fr`

- Run merge octomaps test - pwr d20
  `~/ros_ws/devel/lib/3d_map_server/3d_map_server_test --gtest_filter=MapsIntegratorTest.Test_pwr_d20_m4_t2`

## Rostests

- `rostest 3d_map_server octomaps_integrator.test --text`

## How to debug

- In CMakeLists.txt change `CMAKE_BUILD_TYPE` from `RELEASE` to `RELWITHDEBINFO`.

- Then to run specific test
  `gdb --args ~/ros_ws/devel/lib/3d_map_server/3d_map_server_test --gtest_filter=MapsIntegratorTest.Test_pwr_d20_m1`

- In gdb:
  - `run`
  - `bt`

## Running maps merging tool with ROS

- `roslaunch 3d_map_server merge_maps.launch map1:=/home/mdrwiega/ros_ws/src/3d_map_server/octomaps_dataset/fr_079_t1/scene.ot map2:=/home/mdrwiega/ros_ws/src/3d_map_server/octomaps_dataset/fr_079_t1/model.ot`


## Other

- Add to maps_integrator node
<!--     launch-prefix = "valgrind --leak-check=full" -->
          <!-- launch-prefix="gdb -ex run --args" -->

## Info

- Used datasets are from http://ais.informatik.uni-freiburg.de/projects/datasets/octomap/