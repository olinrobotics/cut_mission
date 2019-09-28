#!/usr/bin/env python
"""Script to generate behaviors file from mission file.

This script generates a behaviors.yaml file from a given mission .yaml
file containing the behaviors that are used by the mission.

The scripts' behavior is controlled by the three global variables at
the top of the file:

    mission_file: the .yaml mission file from which to gen behaviors
    behavior_file: the .yaml output file to which to write behaviors
    behaviors: behaviors and priorities to include in output file
               regardless of their inclusion in the mission file

The script prompts the user in the instance of a file overwrite.
"""

import rospy
import yaml
import os

mission_file = '../config/p2p_test.yaml'
behavior_file = '../config/behaviors_autogen.yaml'
behaviors = {'safety':'0', 'teleop':'1'}

# Load mission behaviors
with open(mission_file, 'r') as f:
    doc = yaml.load(f)
    print("Loading mission behaviors from file: " + doc['title'])
    for i in range(len(doc['waypoints'])):
        behavior = doc['waypoints'][i]['behavior']
        if not behavior in behaviors:
            print("Loading new behavior: " + behavior)
            behaviors[behavior] = '2'

overwrite = False
confirm_ans = ['y', 'Y', 'yes', 'Yes']
deny_ans = ['n', 'N', 'no,' 'No']

# Get user input on overwriting files
if os.path.isfile(behavior_file):
    while(True):
        input = raw_input(
            behavior_file
            + ' already exists - would you like to overwrite it?\n Input: ')
        if input in confirm_ans:
            print("Overwriting " + behavior_file)
            overwrite = True
            break
        elif input in deny_ans:
            print("Canceling operation")
            break
        else:
            print("Erroneous response: please use y or n")

# Regenerate new file to replace overwritten file
if overwrite:
    os.remove(behavior_file)

    # Generate yaml formatted behaviors
    data = dict()
    for behavior in behaviors:
        param = '/behaviors/' + behavior
        data[param] = behaviors[behavior]

    # Write header comments, dump data into file
    with open(behavior_file, "w") as f:
        f.write('# Autogen behavior file from GenerateBehaviors.py\n')
        f.write('# behaviors/[name_of_behavior]: \'[priority]\'\n')
        yaml.dump(data, f, default_flow_style=False)
