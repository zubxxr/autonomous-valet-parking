#!/usr/bin/env python3

import sys
import yaml

def generate_ros2_command(data):
    header_sec = data['header']['stamp']['sec']
    header_nanosec = data['header']['stamp']['nanosec']
    frame_id = data['header']['frame_id']
    pose_position_x = data['pose']['position']['x']
    pose_position_y = data['pose']['position']['y']
    pose_position_z = data['pose']['position']['z']
    pose_orientation_x = data['pose']['orientation']['x']
    pose_orientation_y = data['pose']['orientation']['y']
    pose_orientation_z = data['pose']['orientation']['z']
    pose_orientation_w = data['pose']['orientation']['w']

    command = f"ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{{header: {{stamp: {{sec: {header_sec}, nanosec: {header_nanosec}}}, frame_id: \'{frame_id}\'}}, pose: {{position: {{x: {pose_position_x}, y: {pose_position_y}, z: {pose_position_z}}}, orientation: {{x: {pose_orientation_x}, y: {pose_orientation_y}, z: {pose_orientation_z}, w: {pose_orientation_w}}}}}}}' --once"
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
