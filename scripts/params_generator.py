#!/usr/bin/env python
import argparse
import yaml
import itertools
import roslib
from copy import deepcopy
from collections import defaultdict

class MultiValueParameter:
    namespaces_list = []
    values = []
    
    def __init__(self, namespaces, values):
        self.namespaces_list = namespaces
        self.values = values
    
    def GetValues(self):
        return self.values

    def GetNamespaces(self):
        return self.namespaces_list

class SingleValueParameter:
    namespaces_list = []
    value = 'Inf'
    
    def __init__(self, namespaces, value):
        self.namespaces_list = namespaces
        self.value = value
    
    def GetValue(self):
        return self.value

    def GetNamespaces(self):
        return self.namespaces_list
    
class ParametersGenerator:
    """ It generates parametrs sets based on configuration stored in yaml """

    load_yaml_path = "/code/eval/params_generator_cfg.yaml"
    out_dir = "/code/eval/params/"
    out_pattern = "pset"
    params_lists = []

    def __init__(self):
        self.ParseInputArguments()

    def ParseInputArguments(self):
        parser = argparse.ArgumentParser(description='Description of your program')
        parser.add_argument('-i','--load_yaml_path', help='Path to input configuration file', required=False)
        parser.add_argument('-o','--out_dir', help='Destination directory where generated files will be placed.', required=False)
        parser.add_argument('-op','--pattern', help='Output files names pattern.', required=False)
        args = vars(parser.parse_args())
    
        if args['load_yaml_path']:
            self.load_yaml_path = args['load_yaml_path']
        if args['out_dir']:
            self.out_dir = args['out_dir']
        if args['pattern']:
            self.out_pattern = args['pattern']
    
    def PrintCurrentConfig(self):
        print ("Input path: " + self.load_yaml_path)
        print ("Output directory: " + self.out_dir)
    
    def LoadYamlFile(self, file_path):
        print ("Loading of yaml file: " + file_path)
        with open(file_path, 'r') as stream:
            try:
                return yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
            return []

    def RecursiveExtractParameterFromNestedDict(self, d, namespaces):
        for key, val in d.iteritems():
            if isinstance(val, dict):
                ns = deepcopy(namespaces)
                ns.append(key)
                self.RecursiveExtractParameterFromNestedDict(val, ns)
            else:
                ns = deepcopy(namespaces)
                ns.append(key)
                self.params_lists.append(MultiValueParameter(ns, val))

    def ParseParameters(self):
         dict = self.LoadYamlFile(self.load_yaml_path)
         self.RecursiveExtractParameterFromNestedDict(dict, [])

    def PrintMvParameters(self):
        print("Current set of parameters with values [param namespaces] [values]")
        for i in self.params_lists:
             print "{0}: {1}".format(i.GetNamespaces(), i.GetValues())

    def AddValueToNestedDict(self, vividict, keys, val):
        temp = vividict
        for i, data in enumerate(keys):
            if i == (len(keys) - 1):
                temp.setdefault(keys[i], {})
                temp[keys[i]] = val
            else:
                temp.setdefault(keys[i], {})
                temp = temp[keys[i]]

    def SaveYamlToFile(self, data, file_path):
        with open(file_path, 'w') as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

    def PrintListOfSvParamsLists(self, listoflists):
        print("Single value parameters")
        for l in listoflists:
            print("New list")
            for i in l:
                print "{0}: {1}".format(i.GetNamespaces(), i.GetValue())
        
    def PrepareParametersSets(self):
        list_of_params_list = []
        for mv_param in self.params_lists:
            param_variants = []
            for val in mv_param.GetValues():
                param_variants.append(SingleValueParameter(mv_param.GetNamespaces(), val))
            list_of_params_list.append(param_variants)

        permutations = list(itertools.product(*list_of_params_list))
        return permutations

    def GenerateParametersFiles(self):
        permutations = self.PrepareParametersSets()
        cnt = 0
        for perm in permutations:
            cnt = cnt + 1
            yaml_dict = {}
            for param in perm:
                self.AddValueToNestedDict(yaml_dict, param.GetNamespaces(), param.GetValue())
            self.SaveYamlToFile(yaml_dict, self.out_dir + "/" + self.out_pattern + str(cnt) + ".yaml")

if __name__ == '__main__':
    generator = ParametersGenerator()
    generator.PrintCurrentConfig()
    generator.ParseParameters()
    generator.PrintMvParameters()
    generator.GenerateParametersFiles()