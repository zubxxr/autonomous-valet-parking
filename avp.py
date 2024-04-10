import rclpy
from rclpy.node import Node
# from autoware_adapi_v1_msgs.msg import RouteState
from tier4_planning_msgs.msg import RouteState
from std_msgs.msg import String

import subprocess
import time

class ParkingSpotSubscriber(Node):
    def __init__(self):
        super().__init__('parking_spot_subscriber')
        self.available_parking_spots = None

        self.subscription = self.create_subscription(
            String,
            'available_parking_spots',
            self.available_parking_spots_callback,
            1)

    def available_parking_spots_callback(self, msg):
        self.available_parking_spots = msg.data

class RouteStateSubscriber(Node):

    def __init__(self):
        super().__init__('route_state_subscriber')
        self.subscription = self.create_subscription(
            RouteState,
            '/planning/mission_planning/route_selector/main/state',
            self.route_state_callback,
            10)
        self.flag = False
        self.state = -1

    def route_state_callback(self, msg):
        if msg.state == 6:
           print("\nCar has been parked.")
           self.state = 6
    
def run_ros2_command(command):
    try:
        subprocess.run(command, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")
    
def main(args=None):
    rclpy.init(args=args)

    route_state_subscriber = RouteStateSubscriber()
    parking_spot_subscriber = ParkingSpotSubscriber()
   	
    set_initial_pose_entrance = "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: 1707959787, nanosec: 781504725}, frame_id: \"map\"}, pose: {pose: {position: {x: 3728.91259765625, y: 73723.5546875, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9749977244098511, w: 0.2222148451287896}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'"
    engage_auto_mode = "ros2 topic pub --once /autoware/engage autoware_auto_vehicle_msgs/msg/Engage '{engage: true}'"
    set_goal_pose_entrance = "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708315201, nanosec: 354400841}, frame_id: 'map'}, pose: {position: {x: 3730.5029296875, y: 73724.484375, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9714600644596665, w: 0.2372031685286277}}}' --once"

    parking_spot_locations = {
        1: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395027, nanosec: 711596662}, frame_id: 'map'}, pose: {position: {x: 3715.5400390625, y: 73749.515625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5134630384636576, w: 0.858111710753133}}}' --once",
        2: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395069, nanosec: 522440653}, frame_id: 'map'}, pose: {position: {x: 3718.418701171875, y: 73751.0625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5151419884004101, w: 0.8571048546046579}}}' --once",
        3: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395240, nanosec: 106533260}, frame_id: 'map'}, pose: {position: {x: 3721.328125, y: 73752.4453125, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5121239041168831, w: 0.8589115826626635}}}' --once",
        4: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395289, nanosec: 925354931}, frame_id: 'map'}, pose: {position: {x: 3724.17578125, y: 73754.0390625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.530377158379641, w: 0.8477618001945696}}}' --once",
        5: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395313, nanosec: 171953190}, frame_id: 'map'}, pose: {position: {x: 3727.10595703125, y: 73755.171875, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5100680672684937, w: 0.8601340399920139}}}' --once"
    }

    initiate_parking = True
    print("Park your car? (yes/no)")

    user_input = input().lower()

    if user_input == "yes" or user_input == "y":
        initiate_parking = True
        run_ros2_command(set_initial_pose_entrance)

    counter = 0
    new_counter = 0

    chosen_parking_spot = None

    while rclpy.ok():

        ############### START PARKING SPOT DETECTION ###############
        if initiate_parking and route_state_subscriber.state != 6:

            rclpy.spin_once(parking_spot_subscriber, timeout_sec=0.1)  

            if parking_spot_subscriber.available_parking_spots is None:
                print('Waiting for available parking spots...')

            if parking_spot_subscriber.available_parking_spots is not None:
                
                first_spot_in_queue = int(parking_spot_subscriber.available_parking_spots.split(',')[0].strip('[a]'))

                if first_spot_in_queue != chosen_parking_spot:
                    
                    if counter == 1:
                        print('\nParking spot #%s was taken.' % chosen_parking_spot)
                        counter = 0

                    # Get the first value from the priority queue
                    first_spot_in_queue = int(parking_spot_subscriber.available_parking_spots.split(',')[0].strip('[a]'))

                    # Printing the first value
                    print('\nAvailable parking spot found: %s' % first_spot_in_queue)
                    
                    parking_spot_goal_pose_command = parking_spot_locations[first_spot_in_queue]
                    run_ros2_command(parking_spot_goal_pose_command)
                    run_ros2_command(engage_auto_mode)
                    print('Setting goal pose to parking spot:', first_spot_in_queue)

                    counter += 1
                    chosen_parking_spot = first_spot_in_queue
                
            time.sleep(1)
        ############### END PARKING SPOT DETECTION #################

        if new_counter == 0:
            if route_state_subscriber.state == 6:
                print("Retrieve car? (yes/no)")

                user_input = input().lower()

                if user_input == "yes" or user_input == "y":
                    print("Going to Drop Off Zone.")
                    run_ros2_command(set_goal_pose_entrance)
                    run_ros2_command(engage_auto_mode)
                elif user_input == "no" or user_input == "n":
                    print("Exiting the script.")
                    exit()
                else:
                    print("Invalid input. Please enter 'yes' or 'no'.")

                route_state_subscriber.state = -1
                new_counter += 1                  
        
        rclpy.spin_once(route_state_subscriber, timeout_sec=1)
        
    route_state_subscriber.destroy_node()
    parking_spot_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
