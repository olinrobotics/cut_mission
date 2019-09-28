### Overview
This folder holds launch files for cut\_mission

### Naming Convention

These naming conventions create standardized, predictable names and ensure that the files in the /launch folder are naturally grouped.

_Behavior Launch Files:_ "behavior\_<behavior-name>.yaml" (ex. behavior\_cut.yaml)

_Mission Launch Files:_ "mission\_<mission-name>.yaml" (ex. mission\_p2p-test.yaml)

### Files

#### `behavior_cut.launch`
Launches CutBehavior to pilot the tractor between two waypoints while executing a blade path provided by calling the CutPlanner service, which is also launched. By default, launch file executes Pathing node to allow waypoint navigation and arrival detection. If this launch file is to be included in a higher-level launch file, pass '1' as argument 'included' to prevent re-launching of Pathing node.

#### `behavior_scan.launch`
Launches ScanBehavior to pilot the tractor between two waypoints while scanning a surface and saving points to a file. By default, launch file executes Pathing node to allow waypoint navigation and arrival detection. If this launch file is to be included in a higher-level launch file, pass '1' as argument 'included' to prevent re-launching of Pathing node.

#### `behavior_p2p.launch`
Launches P2PBehavior to pilot the tractor between two waypoints. By default, launch file executes Pathing node to allow waypoint navigation and arrival detection. 

#### `mission_ex.launch`
Loads behaviors from `behaviors_ex.yaml` file in `/cut_mission/config`, starts state controller using loaded behaviors, and starts MissionPlanner node with a param representing the location of a mission .yaml file. The behavior .yaml file can be generated from the mission .yaml file by running the script `GenerateBehaviors.py`. Not meant to be run, only used as a template.

#### `mission_p2p-test.launch`
Mission file based off of `mission_ex.launch` template. In addition to basic aspects, also includes necessary behavior launch files for P2PBehavior.

