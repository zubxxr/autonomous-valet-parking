#!/usr/bin/env python3

import sys
import yaml

def generate_ros2_command(data):
    header_sec = data['header']['stamp']['sec']
    header_nanosec = data['header']['stamp']['nanosec']
    frame_id = data['header']['frame_id']
    pose_position_x = data['pose']['pose']['position']['x']
    pose_position_y = data['pose']['pose']['position']['y']
    pose_position_z = data['pose']['pose']['position']['z']
    pose_orientation_z = data['pose']['pose']['orientation']['z']
    pose_orientation_w = data['pose']['pose']['orientation']['w']
    covariance = data['pose']['covariance']

    command = "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: " + str(header_sec) + ", nanosec: " + str(header_nanosec) + "}, frame_id: '" + frame_id + "'}, pose: {pose: {position: {x: " + str(pose_position_x) + ", y: " + str(pose_position_y) + ", z: " + str(pose_position_z) + "}, orientation: {x: 0.0, y: 0.0, z: " + str(pose_orientation_z) + ", w: " + str(pose_orientation_w) + "}}, covariance: " + str(covariance) + "}}'"
    return command

def main():
    if len(sys.argv) != 2:
        print("Usage: python script.py '<pose_data_as_string>'")
        sys.exit(1)

    pose_data_string = sys.argv[1]

    # Split the string into lines
    pose_data_lines = pose_data_string.splitlines()
    
    try:
        # Join lines into a single string and parse YAML
        data = yaml.safe_load('\n'.join(pose_data_lines))
    except yaml.YAMLError as e:
        print("Error parsing YAML string:", e)
        sys.exit(1)

    command = generate_ros2_command(data)
    print(command)

if __name__ == "__main__":
    main()

