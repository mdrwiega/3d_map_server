# 3d_map_server

The package contains a 3D map server that is able to integrate maps received from multiple robots.

Main features:

- The main purpose of the package is to provide a 3D maps integration and maps storage tools.
- The map server can integrate maps without an initial information about the transformation between maps.\
  It detects overlapping regions on maps (if exist) and based on that estimates the transformation.
- It uses multiple methods for maps alignment, for example SAC, GCC, ICP, OICP and NDT.
- Software is based on the ROS 1 (Noetic) but migration to the ROS 2 is also planned.

![Feature base matching example](design/matching_example.png)

Additional documentation is available in the following [publication](https://ieeexplore.ieee.org/document/9528427).
```
@inproceedings{drwiega21,
  author = {Michał Drwięga},
  title = {Incremental 3D Maps Server based on Feature Matching for Multi-robot Systems},
  journal = {25th International Conference on Methods and Models in Automation and Robotics (MMAR)},
  publisher = {IEEE},
  year = 2021,
  doi = {10.1109/MMAR49549.2021.9528427},
}
```

## Build

- Clone the repository to the ROS workspace (e.g. ros_ws/src)

  `git clone https://gitlab.com/mdrwiega/3d_map_server.git`

- Build package

  `catkin_make`

## Run the maps integration tool with ROS

`roslaunch 3d_map_server merge_maps.launch map1:=~/ros_ws/src/3d_map_server/octomaps_dataset/fr_079_t1/scene.ot map2:=~/ros_ws/src/3d_map_server/octomaps_dataset/fr_079_t1/model.ot`

## Tests

- Compile tests

  `catkin_make tests`

- Compile and run all tests

  `catkin_make run_tests 3d_map_server`

- Run merge octomaps test - fr079

  `~/ros_ws/devel/lib/3d_map_server/3d_map_server_test --gtest_filter=MapsIntegratorTest.Test_fr`

- Run merge octomaps test - pwr d20

  `~/ros_ws/devel/lib/3d_map_server/3d_map_server_test --gtest_filter=MapsIntegratorTest.Test_pwr_d20_m4_t2`

### Integration tests

`rostest 3d_map_server octomaps_integrator.test --text`

## Design (implementation details)

![Class Diagram](design/class_diagram.png)

## For developers

### Debugging

- In CMakeLists.txt change `CMAKE_BUILD_TYPE` from `RELEASE` to `RELWITHDEBINFO`.

- Then to run specific test
  `gdb --args ~/ros_ws/devel/lib/3d_map_server/3d_map_server_test --gtest_filter=MapsIntegratorTest.Test_pwr_d20_m1`

- In gdb:
  - `run`
  - `bt`

- Add to maps_integrator node
<!--     launch-prefix = "valgrind --leak-check=full" -->
          <!-- launch-prefix="gdb -ex run --args" -->

## References

- Tests use datasets from Freiburg University - [link](http://ais.informatik.uni-freiburg.de/projects/datasets/octomap/)