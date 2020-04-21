#!/usr/bin/env python3

import os, sys
from os import listdir, mkdir
from os.path import isfile, join, splitext, exists, isdir
import time
import subprocess
import argparse

class TestsTool:
    def __init__(self):
        self.parameters_dir = '/home/mdrwiega/ros_ws/temp/params'
        self.datasets_dir = '/home/mdrwiega/ros_ws/src/octomap_tools/octomaps_dataset'
        self.output_dir = '/home/mdrwiega/ros_ws/temp/output'

        self.parse_input_parameters()

    def add_parameters_list(self, parser):
        parser.add_argument('-d', '--dir', required=False, help='Set working directory')

    def parse_input_parameters(self):
        parser = argparse.ArgumentParser(description='Accounting checker')
        self.add_parameters_list(parser)
        args = vars(parser.parse_args())

        # if args['dir']:
        #     self.working_dir = args['dir']
        # os.mkdir(self.results_dir)

    def run_tests(self):
      datasets = ['fr_079_t1']

      os.system('roscore&')
      time.sleep(1.0)
      os.system('rosparam load /home/mdrwiega/ros_ws/src/octomap_tools/config/params.yaml' + ' maps_server')

      try:

        # Process each params set
        for pset in listdir(self.parameters_dir):
          print('Processing of parameters set: %s' %pset)
          os.system('rosparam load ' + join(self.parameters_dir, pset) + ' maps_server')

          # Process all tests units
          for dataset in datasets:
            data_dir = join(self.datasets_dir, dataset)
            test_unit = dataset + '_' + pset.replace('.yaml', '')
            test_output_dir = join(self.output_dir, test_unit) + '/'

            # Create test output dir
            try:
              os.mkdir(test_output_dir)
            except OSError:
              pass

            print('  Started processing of dataset: %s with parameters: %s' %(dataset, pset))

            os.system('rosparam set maps_server/output_dir ' + test_output_dir)
            cmd = 'roslaunch octomap_tools merge_maps.launch load_params:=False' \
                + ' map1:=' + join(data_dir, 'scene.ot') \
                + ' map2:=' + join(data_dir, 'model.ot')
            p = subprocess.Popen(cmd, stdin=subprocess.PIPE, shell=True)
            try:
              p.wait()
            except Exception as e:
              print("Process has been closed. Exception: %s" %str(e))

            os.system('rosparam delete /')
            print('  Finished processing of dataset: %s with parameters: %s' %(dataset, pset))
            print('  Output saved to %s' %test_output_dir)

        os.system('killall -9 roscore rosmaster')

      except:
        os.system('killall -9 roscore rosmaster')

if __name__ == '__main__':
    tests_tool = TestsTool()
    tests_tool.run_tests()
