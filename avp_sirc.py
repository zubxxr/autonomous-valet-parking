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
            '/parking_spots/empty',
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
   	
    engage_auto_mode = "ros2 topic pub --once /autoware/engage autoware_vehicle_msgs/msg/Engage '{engage: True}' -1"

    # set_goal_pose_entrance = "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708315201, nanosec: 354400841}, frame_id: 'map'}, pose: {position: {x: 3730.5029296875, y: 73724.484375, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9714600644596665, w: 0.2372031685286277}}}' --once"

    parking_spot_locations = {
        1: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736234930, nanosec: 485255271}, frame_id: 'map'}, pose: {position: {x: -44.340084075927734, y: -14.542484283447266, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6048959869666175, w: 0.79630449261051}}}' --once",
        2: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736234964, nanosec: 268967027}, frame_id: 'map'}, pose: {position: {x: -41.737525939941406, y: -13.785665512084961, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6017180648737738, w: 0.7987085641237115}}}' --once",
        3: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235015, nanosec: 500375897}, frame_id: 'map'}, pose: {position: {x: -38.7567138671875, y: -12.567403793334961, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6019133113169981, w: 0.7985614351190562}}}' --once",
        4: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235477, nanosec: 950428247}, frame_id: 'map'}, pose: {position: {x: -35.9503173828125, y: -11.982339859008789, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6007336954096015, w: 0.799449202388447}}}' --once",
        5: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235517, nanosec: 532171546}, frame_id: 'map'}, pose: {position: {x: -32.98591232299805, y: -11.230877876281738, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.603871811065959, w: 0.7970814486612511}}}' --once",
        6: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235543, nanosec: 363285134}, frame_id: 'map'}, pose: {position: {x: -30.08383560180664, y: -10.155061721801758, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.608585037220489, w: 0.7934886593211878}}}' --once",
        7: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235564, nanosec: 777861615}, frame_id: 'map'}, pose: {position: {x: -27.20238494873047, y: -9.49226188659668, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6007326637076104, w: 0.7994499776438543}}}' --once",
        8: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235588, nanosec: 16728058}, frame_id: 'map'}, pose: {position: {x: -24.6075382232666, y: -8.993618965148926, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.6036992773480252, w: 0.797212131449009}}}' --once",
        15: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235678, nanosec: 86261653}, frame_id: 'map'}, pose: {position: {x: -42.034812927246094, y: -22.750713348388672, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7969156099507567, w: 0.6040906476819629}}}' --once",
        16: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235705, nanosec: 118039164}, frame_id: 'map'}, pose: {position: {x: -39.109901428222656, y: -22.36135482788086, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7978757289989354, w: 0.6028219646582376}}}' --once",
        17: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235729, nanosec: 260929592}, frame_id: 'map'}, pose: {position: {x: -36.33027648925781, y: -21.73630142211914, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7999696817030807, w: 0.6000404222682599}}}' --once",
        18: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235765, nanosec: 496677516}, frame_id: 'map'}, pose: {position: {x: -33.3206672668457, y: -20.794353485107422, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7897854690370711, w: 0.6133831697217438}}}' --once",
        19: "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1736235786, nanosec: 558316281}, frame_id: 'map'}, pose: {position: {x: -30.4317569732666, y: -19.839237213134766, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7983271798448894, w: 0.6022239732200185}}}' --once",
    }

    initiate_parking = True
    print("Park your car? (yes/no)")

    user_input = input().lower()

    if user_input == "yes" or user_input == "y":
        initiate_parking = True

    counter = 0
    new_counter = 0

    chosen_parking_spot = None

    while rclpy.ok():

        ############### START PARKING SPOT DETECTION ###############
        if initiate_parking and route_state_subscriber.state != 6:

            rclpy.spin_once(parking_spot_subscriber, timeout_sec=0.1)  

            parking_spots = parking_spot_subscriber.available_parking_spots

            if parking_spots is None:
                print('Waiting for available parking spots...')

            if parking_spots is not None:
                
                first_spot_in_queue = int(parking_spots.split(': ')[1].split(',')[0])

                if first_spot_in_queue != chosen_parking_spot:
                    
                    if counter == 1:
                        print('\nParking spot #%s was taken.' % chosen_parking_spot)
                        counter = 0

                    # Get the first value from the priority queue
                    first_spot_in_queue = int(parking_spots.split(': ')[1].split(',')[0])

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

        # if new_counter == 0:
        #     if route_state_subscriber.state == 6:
        #         print("Retrieve car? (yes/no)")

        #         user_input = input().lower()

        #         if user_input == "yes" or user_input == "y":
        #             print("Going to Drop Off Zone.")
        #             run_ros2_command(set_goal_pose_entrance)
        #             run_ros2_command(engage_auto_mode)
        #         elif user_input == "no" or user_input == "n":
        #             print("Exiting the script.")
        #             exit()
        #         else:
        #             print("Invalid input. Please enter 'yes' or 'no'.")

        #         route_state_subscriber.state = -1
        #         new_counter += 1                  
        
        rclpy.spin_once(route_state_subscriber, timeout_sec=1)
        
    route_state_subscriber.destroy_node()
    parking_spot_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
