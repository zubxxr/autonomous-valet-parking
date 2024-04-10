# Autoware Automated Valet Parking Script

This script has been developed for use with Autoware. It communicates with the external parking spot detection node and receives available parking spot messages, which are used to tell the ego vehicle where to park.


<br>

## Running Script on Sample Map Provided by Autoware

To run the script on the default map given by Autoware, you first need to launch the [Parking Spot Detection Node](https://github.com/zubxxr/Parking-Spot-Detection-Autoware). Once it is running, launch Autoware in another terminal.

```
source /path/to/your/autoware/install/setup.bash
source /opt/ros/<distro>/setup.bash

ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=$HOME/autoware_map/sample-map-planning \
    vehicle_model:=sample_vehicle \
    sensor_model:=sample_sensor_kit
```



## Customizing Map Parameters

To run the script on your custom map, you need to know the coordinates of the entrance to the parking lot and each parking spot and then update the initial pose and goal pose commands inside the script. Follow the steps below to customize the map parameters.

## Generating a Publish Initial Pose Command

To automate the process of publishing the intitial pose of the ego vehicle, the publish command is required. Follow these steps:

#### 1. Ensure Autoware is running.
Before proceeding, ensure that Autoware is running. If not, start Autoware with the following command:

```
source /path/to/your/autoware/install/setup.bash
source /opt/ros/<distro>/setup.bash

ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path:=/PATH/TO/YOUR/MAP \
    vehicle_model:=YOUR_VEHICLE \
    sensor_model:=YOUR_SENSOR_KIT
```

#### 2. Source Autoware and ROS 2 Environment.
Open a new terminal and source the Autoware and ROS 2 environment setup files. Modify the paths according to your installation directory if they differ.

```
source /path/to/your/autoware/install/setup.bash
source /opt/ros/<distro>/setup.bash
```

#### 3. Run the command inside the sourced terminal: 
```
ros2 topic echo /initialpose
```

#### 4. Set an initial pose at the parking spot entrance on a lane inside the RViz window.

#### 5. Capture the coordinates from the echoed message.

### Example Echoed Message:
```
header:
  stamp:
    sec: 1708461407
    nanosec: 151011266
  frame_id: map
pose:
  pose:
    position:
      x: 3730.777099609375
      y: 73724.90625
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.9610824749843778
      w: 0.27626160840388697
  covariance:
  - 0.25
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.25
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.06853891909122467
```

#### 6. Use the obtained coordinates as a command line argument in the generate_initialpose_command.py script and execute the script:
```
python generate_initial_pose_command.py "{Include printed coordinates here}".
```
### Example Script Output

```
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: 1708461407, nanosec: 151011266}, frame_id: "map"}, pose: {pose: {position: {x: 3730.777099609375, y: 73724.90625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9610824749843778, w: 0.27626160840388697}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'
```
#### 7. Update variable '*set_initial_pose_entrance*' with above output.

### Example Variable
```
set_initial_pose_entrance = "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: 1707959787, nanosec: 781504725}, frame_id: \"map\"}, pose: {pose: {position: {x: 3728.91259765625, y: 73723.5546875, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9749977244098511, w: 0.2222148451287896}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'"
```

<br>


## Generating Publish Goal Pose Commands

Similarly to the initial pose, the goal pose location coordinates are also required. Follow these steps:

#### 1. Run the command inside the sourced terminal: 
``` 
ros2 topic echo /planning/mission_planning/goal
```

#### 2. Select a goal pose in the center of a parking spot inside the RViz window.

#### 3. Capture the coordinates from the echoed message.

### Example Echoed Message:
```
header:
  stamp:
    sec: 1708463402
    nanosec: 486667092
  frame_id: map
pose:
  position:
    x: 3733.503173828125
    y: 73758.703125
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: -0.5108924948741428
    w: 0.859644611849149
```

#### 4. Use the obtained coordinates as a command line argument in the generate_goal_pose_command.py script and execute the script:
```
python generate_goal_pose_command.py "{Include printed coordinates here}".
```
### Example Script Output

```
ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708463402, nanosec: 486667092}, frame_id: 'map'}, pose: {position: {x: 3733.503173828125, y: 73758.703125, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5108924948741428, w: 0.859644611849149}}}' --once
```

#### 5. Update variable '*parking_spot_locations*' with above output.

### Example Variable
```
parking_spot_locations = {
    1: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395027, nanosec: 711596662}, frame_id: 'map'}, pose: {position: {x: 3715.5400390625, y: 73749.515625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5134630384636576, w: 0.858111710753133}}}' --once"
}
```
#### 6. Repeat for all other parking spots and add to array.

### Example Complete Variable for 5 Parking Spots

```
parking_spot_locations = {
    1: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395027, nanosec: 711596662}, frame_id: 'map'}, pose: {position: {x: 3715.5400390625, y: 73749.515625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5134630384636576, w: 0.858111710753133}}}' --once",
    2: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395069, nanosec: 522440653}, frame_id: 'map'}, pose: {position: {x: 3718.418701171875, y: 73751.0625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5151419884004101, w: 0.8571048546046579}}}' --once",
    3: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395240, nanosec: 106533260}, frame_id: 'map'}, pose: {position: {x: 3721.328125, y: 73752.4453125, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5121239041168831, w: 0.8589115826626635}}}' --once",
    4: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395289, nanosec: 925354931}, frame_id: 'map'}, pose: {position: {x: 3724.17578125, y: 73754.0390625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.530377158379641, w: 0.8477618001945696}}}' --once",
    5: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395313, nanosec: 171953190}, frame_id: 'map'}, pose: {position: {x: 3727.10595703125, y: 73755.171875, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5100680672684937, w: 0.8601340399920139}}}' --once"
}
```


## Executing Parking Spot Detection
Now that the avp script has been updated, the parking spot detection can be executed, which will send available parking spots to the avp script, allowing the ego to park itself. 

#### 1. Launch Parking Spot Detection

Follow the steps [here](https://github.com/zubxxr/Parking-Spot-Detection-Autoware) to launch the Parking Spot Detection node.

#### 2. Launch Autoware

#### 3. Execute AVP Script

<br> 
*Note: README.md is a work in progress. More detailed documentation will be added soon.*

