# octomap_tools

The set of octomaps related tools.

### Compilation

Clone and install md\_utils.
`git clone https://github.com/mdrwiega/md_utils.git`
`mkdir md_utils/build && cd md_utils/build`
`cmake .. && make && sudo make install`

Clone and install md\_octomap (modified octomap lib)
`git clone git@gitlab.com:mdrwiega/md_octomap.git`
`mkdir md_octomap/build && cd md_octomap/build`
`cmake .. && make && sudo make install`

Clone octomap_tools repository to ros workspace i.e. catkin_ws/src
`git clone https://gitlab.com/mdrwiega/octomap_tools.git`

Compile projects in workspace
`catkin_make`

### Tests

Compile tests.
`catkin_make tests`

Compile and run all tests
`catkin_make run_tests octomap_tools`

Run merge octomaps test - fr079
`~/catkin_ws/devel/lib/octomap_tools/octomap_tools_test --gtest_filter=MapsIntegratorTest.Test_fr`

Run merge octomaps test - pwr d20
`~/catkin_ws/devel/lib/octomap_tools/octomap_tools_test --gtest_filter=MapsIntegratorTest.Test_pwr_d20_m4_t2`

### Rostests

`rostest octomap_tools octomaps_integrator.test --text`