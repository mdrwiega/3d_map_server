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


Clone octomap_tools repository.

`git clone https://gitlab.com/mdrwiega/octomap_tools.git`

Make build directory.

`mkdir octomap_tools/build && cd octomap_tools/build`
  
Generate project and compile it.

`cmake -j8 --build build`

### Run UT

	cd build

	test/octomapToolsTests

	test/octomapToolsTests --gtest_filter=IntegrateOctomaps.EstimationOnly_Demo_PclVis


### Installation

`sudo make install`
