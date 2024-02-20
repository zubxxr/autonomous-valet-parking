import rclpy
from rclpy.node import Node
from autoware_adapi_v1_msgs.msg import RouteState
import subprocess
import time

class RouteStateSubscriber(Node):

    def __init__(self):
        super().__init__('route_state_subscriber')
        self.subscription = self.create_subscription(
            RouteState,
            '/planning/mission_planning/route_state',
            self.route_state_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.flag = False
        self.state = -1

    def route_state_callback(self, msg):
        self.get_logger().info('Received route state: %s' % msg.state)
        if msg.state == 3:
           print("state is 3")
           self.state = 3
        
def run_ros2_command(command):
    try:
        subprocess.run(command, shell=True, check=True)
        print("Command executed successfully.")
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")
    

if __name__ == '__main__':
    rclpy.init()

    route_state_subscriber = RouteStateSubscriber()
   	
    # Define your ROS2 commands
    ros2_commands = [
        "ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {stamp: {sec: 1707959787, nanosec: 781504725}, frame_id: \"map\"}, pose: {pose: {position: {x: 3728.91259765625, y: 73723.5546875, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9749977244098511, w: 0.2222148451287896}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}'",
        "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395910, nanosec: 36953338}, frame_id: 'map'}, pose: {position: {x: 3734.23876953125, y: 73757.765625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5527481665047441, w: 0.8333483451868394}}}' --once",
        
        "ros2 topic pub --once /autoware/engage autoware_auto_vehicle_msgs/msg/Engage '{engage: true}'"
        # Add more commands here...
    ]

    ros2_commands_2 = [
        "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708315201, nanosec: 354400841}, frame_id: 'map'}, pose: {position: {x: 3730.5029296875, y: 73724.484375, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.9714600644596665, w: 0.2372031685286277}}}' --once",
        "ros2 topic pub --once /autoware/engage autoware_auto_vehicle_msgs/msg/Engage '{engage: true}'"
    ]   

    # parking spots for default map from left to right
    parking_spot_locations = [
        "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395027, nanosec: 711596662}, frame_id: 'map'}, pose: {position: {x: 3715.5400390625, y: 73749.515625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5134630384636576, w: 0.858111710753133}}}' --once",
        "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395069, nanosec: 522440653}, frame_id: 'map'}, pose: {position: {x: 3718.418701171875, y: 73751.0625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5151419884004101, w: 0.8571048546046579}}}' --once",
        "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395240, nanosec: 106533260}, frame_id: 'map'}, pose: {position: {x: 3721.328125, y: 73752.4453125, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5121239041168831, w: 0.8589115826626635}}}' --once",
        "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395289, nanosec: 925354931}, frame_id: 'map'}, pose: {position: {x: 3724.17578125, y: 73754.0390625, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.530377158379641, w: 0.8477618001945696}}}' --once",
        "ros2 topic pub /planning/mission_planning/goal geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 1708395313, nanosec: 171953190}, frame_id: 'map'}, pose: {position: {x: 3727.10595703125, y: 73755.171875, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.5100680672684937, w: 0.8601340399920139}}}' --once"
    ]

    

    # set initial pose at entrance, set goal to end of parking lot, and enable autonomous mode
    for command in ros2_commands[:3]:
        run_ros2_command(command)
        time.sleep(2)

    counter = 0

    initiate_parking = False
    running = True

    while running and rclpy.ok():


        ############### START PARKING SPOT DETECTION ###############

        if initiate_parking:
            run_ros2_command(parking_spot_locations[0])
            run_ros2_command(ros2_commands[2])
            running = False

        ############### END PARKING SPOT DETECTION #################
            
            

        # if counter is 0, drive back to entrance
        if counter == 0:
            if route_state_subscriber.state == 3:
                print("Going to Entrance.")
                for command in ros2_commands_2:
                    run_ros2_command(command)
                    time.sleep(2)  

                route_state_subscriber.state = -1
                counter += 1       

        # if counter is 1, drive back to goal
        if counter == 1:
            if route_state_subscriber.state == 3:
                print("Going to Goal.")
                for command in ros2_commands[1:]:
                    run_ros2_command(command)
                    time.sleep(2) 
                
                route_state_subscriber.state = -1
                counter += 1

        if counter == 2:
            counter = 0
        

        rclpy.spin_once(route_state_subscriber, timeout_sec=1)
    

    route_state_subscriber.destroy_node()
    rclpy.shutdown()

