#!/usr/bin/env python
import rospy
import yaml
import os

mission_file = '../config/single_cut.yaml'
behavior_file = '../config/behaviors.yaml'
behaviors = []

with open(mission_file, 'r') as f:
    doc = yaml.load(f)
    print("Loading mission behaviors: " + doc['title'])
    for i in range(len(doc['waypoints'])):
        behavior = doc['waypoints'][i]['behavior']
        if not behavior in behaviors:
            print("New behavior: " + behavior)
            behaviors.append(behavior)

overwrite = False

if os.path.isfile(behavior_file):
    while(True):
        input = raw_input(behavior_file + ' already exists - would you like to overwrite it?\n Input: ')
        if input == 'y' or input == 'Y' or input == 'Yes' or input == 'yes':
            print("Overwriting " + behavior_file)
            overwrite = True
            break
        elif input == 'n' or input == 'N' or input == 'No' or input == 'no':
            print("Canceling operation")
            break
        else:
            print("Erroneous response: please use y or n")

if overwrite:
    os.remove(behavior_file)
    data = dict()
    index = 0
    for behavior in behaviors:
        param = '/behaviors/' + behavior
        data[param] = index
        index += 1
    with open(behavior_file, "w") as f:
        f.write('# Autogen behavior file from GenerateBehaviors.py\n')
        f.write('# behaviors/[name_of_behavior]: \'[priority]\'\n')
        yaml.dump(data, f, default_flow_style=False)
