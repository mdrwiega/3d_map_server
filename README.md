# octomap_tools

The set of octomaps related tools.

## Compilation

Clone octomap_tools repository to ros workspace i.e. catkin_ws/src
`git clone https://gitlab.com/mdrwiega/octomap_tools.git`

Compile projects in ros workspace
`catkin_make`

## Tests

Compile tests.
`catkin_make tests`

Compile and run all tests
`catkin_make run_tests octomap_tools`

Run specific test
`~/ros_ws/devel/lib/octomap_tools/octomap_tools_test --gtest_filter=MapsIntegratorTest.TestName`

Run merge octomaps test - fr079
`~/ros_ws/devel/lib/octomap_tools/octomap_tools_test --gtest_filter=MapsIntegratorTest.Test_fr`

Run merge octomaps test - pwr d20
`~/ros_ws/devel/lib/octomap_tools/octomap_tools_test --gtest_filter=MapsIntegratorTest.Test_pwr_d20_m4_t2`

## Rostests

`rostest octomap_tools octomaps_integrator.test --text`

## How to debug
In CMakeLists.txt change `CMAKE_BUILD_TYPE` from `RELEASE` to `RELWITHDEBINFO`.

Then to run specific test
`gdb --args ~/ros_ws/devel/lib/octomap_tools/octomap_tools_test --gtest_filter=MapsIntegratorTest.Test_pwr_d20_m1`

In gdb:
- `run`
- `bt`

## Running maps merging tool with ROS
`roslaunch octomap_tools merge_maps.launch map1:=/home/mdrwiega/ros_ws/src/3d_map_server/octomaps_dataset/fr_079_t1/scene.ot map2:=/home/mdrwiega/ros_ws/src/3d_map_server/octomaps_dataset/fr_079_t1/model.ot`

## Batch processing
### Pipeline
- generate params sets
- start tests
- analyse

### Generate parameters sets
```
rosrun octomap_tools params_generator.py \
  -i /home/mdrwiega/ros_ws/src/3d_map_server/config/params_generator_cfg.yaml \
  -o /home/mdrwiega/ros_ws/temp/params
```

### Process params sets
`rosrun octomap_tools tests.py`

## Other
- Add to maps_integrator node
<!--     launch-prefix = "valgrind --leak-check=full" -->

