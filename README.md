# Autoware Automated Valet Parking Script

This script has been developed for use with Autoware. It communicates with the external parking spot detection node and receives available parking spot messages, which are used to tell the ego vehicle where to park.


## Table of Contents

- [Quick Setup: Running Script on Sample Map Provided by Autoware](#quick-setup-running-script-on-sample-map-provided-by-autoware)
- [Detailed and Customized Setup](#detailed-and-customized-setup)
    - [Generating a Publish Initial Pose Command](#generating-a-publish-initial-pose-command)
    - [Generating Publish Goal Pose Commands](#generating-publish-goal-pose-commands)
    - [Executing Parking Spot Detection](#executing-parking-spot-detection)

## Quick Setup: Running Script on Sample Map Provided by Autoware

This section provides a quick setup guide to use the parking spot detection node and Autoware, enabling the vehicle to park itself on the default map provided by Autoware.

1. **Launch Parking Spot Detection Node**: Start by launching the [Parking Spot Detection Node](https://github.com/zubxxr/Parking-Spot-Detection-Autoware) in a terminal.

2. **Launch Autoware**: In another terminal, execute the following commands to launch Autoware:

    ```bash
    source /path/to/your/autoware/install/setup.bash
    source /opt/ros/<distro>/setup.bash

    ros2 launch autoware_launch planning_simulator.launch.xml \
        map_path:=$HOME/autoware_map/sample-map-planning \
        vehicle_model:=sample_vehicle \
        sensor_model:=sample_sensor_kit
    ```

    Make sure to replace `/path/to/your/autoware/install/setup.bash` with the actual path to your Autoware installation directory and `<distro>` with your ROS distribution (e.g., `galactic`, `humble`, etc.).


## Detailed and Customized Setup

To run the script on your custom map, you need to know the coordinates of the entrance to the parking lot and each parking spot and then update the initial pose and goal pose commands inside the script. Follow the steps below to customize the map parameters.

### Generating a Publish Initial Pose Command

Automate the process of publishing the initial pose of the ego vehicle by following these steps:

1. **Ensure Autoware is Running**: Confirm that Autoware is operational. If not, initiate Autoware with the provided command.

    ```bash
    source /path/to/your/autoware/install/setup.bash
    source /opt/ros/<distro>/setup.bash

    ros2 launch autoware_launch planning_simulator.launch.xml \
        map_path:=/PATH/TO/YOUR/MAP \
        vehicle_model:=YOUR_VEHICLE \
        sensor_model:=YOUR_SENSOR_KIT
    ```
    Make sure to replace `/path/to/your/autoware/install/setup.bash` with the actual path to your Autoware installation directory and `<distro>` with your ROS distribution (e.g., `galactic`, `humble`, etc.).

2. **Source Autoware and ROS 2 Environment**: In a new terminal, source the Autoware and ROS 2 environment setup files.

    ```bash
    source /path/to/your/autoware/install/setup.bash
    source /opt/ros/<distro>/setup.bash
    ```

3. **Echo the Initial Pose Topic**: Run the following command to echo the initial pose topic.

    ```bash
    ros2 topic echo /initialpose
    ```
 
4. **Capture Coordinates**: Select an initial pose at the parking spot entrance within RViz, and capture the coordinates from the echoed message.
   
    **Example Echoed Coordinates**
    ```yaml
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
5. **Use Generated Coordinates**: Utilize the obtained coordinates as a command line argument in the `generate_initialpose_command.py` script and execute the script.
    ```bash
    python generate_initial_pose_command.py "{Include printed coordinates here}".
    ```
    **Example Script Output**

    ```bash
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: 1708461407, nanosec: 151011266}, frame_id: "map"}, pose: {pose: {position: {x: 3730.777099609375, y: 73724.90625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9610824749843778, w: 0.27626160840388697}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'
    ```


6. **Update Variable**: Update variable `set_initial_pose_entrance` with above output.
    ```python
    set_initial_pose_entrance = "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: 1707959787, nanosec: 781504725}, frame_id: \"map\"}, pose: {pose: {position: {x: 3728.91259765625, y: 73723.5546875, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9749977244098511, w: 0.2222148451287896}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'"
    ```
   
### Generating Publish Goal Pose Commands

Similarly to the initial pose, the goal pose location coordinates are also required. Follow these steps:

1. **Echo the Goal Pose Topic**: Run the command inside the sourced terminal:
   
    ```bash
    ros2 topic echo /planning/mission_planning/goal
    ```
    
3. **Capture Coordinates**: Select a goal pose in the center of a parking spot inside the RViz window, and capture the coordinates from the echoed message.
    **Example Echoed Message**
    ```yaml
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

4. **Use Generated Coordinates**: Use the obtained coordinates as a command line argument in the `generate_goal_pose_command.py` script and execute the script:
    ```bash
    python generate_goal_pose_command.py "{Include printed coordinates here}".
    ```
    **Example Script Output**
    
    ```bash
    ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708463402, nanosec: 486667092}, frame_id: 'map'}, pose: {position: {x: 3733.503173828125, y: 73758.703125, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5108924948741428, w: 0.859644611849149}}}' --once
    ```

5. **Update Variable**: Update variable `parking_spot_locations` with above output.

    **Example Variable**
    ```python
    parking_spot_locations = {
        1: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395027, nanosec: 711596662}, frame_id: 'map'}, pose: {position: {x: 3715.5400390625, y: 73749.515625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5134630384636576, w: 0.858111710753133}}}' --once"
    }
    ```
    
6. Repeat for all other parking spots and add to array.

    **Example Complete Variable for 5 Parking Spots**
    ```python
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

