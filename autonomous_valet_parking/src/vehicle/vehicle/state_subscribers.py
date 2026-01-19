from rclpy.node import Node
from tier4_planning_msgs.msg import RouteState
from autoware_adapi_v1_msgs.msg import MotionState
from std_msgs.msg import String

class ParkingSpotSubscriber(Node):
    def __init__(self, args):
        super().__init__('parking_spot_subscriber')
        
        self.available_parking_spots = None

        self.parking_spots_subscription = self.create_subscription(String, '/avp/parking_spots', self.available_parking_spots_callback, 1)

    def available_parking_spots_callback(self, msg):
        self.available_parking_spots = msg.data

class RouteStateSubscriber(Node):
    def __init__(self, args):
        super().__init__(f'route_state_subscriber')
        
        self.subscription = self.create_subscription(
            RouteState,
            '/planning/mission_planning/route_selector/main/state',
            self.route_state_callback,
            10)
        
        self.flag = False
        self.state = -1

    def route_state_callback(self, msg):
        # 6 - arrived
        self.state = msg.state

class MotionStateSubscriber(Node):
    def __init__(self, args):
        super().__init__(f'motion_state_subscriber')

        self.subscription = self.create_subscription(MotionState, '/api/motion/state', self.motion_state_callback, 10)
        
        self.flag = False
        self.state = -1

    def motion_state_callback(self, msg):
        # 1 - stopped
        # 3 - moving
        self.state = msg.state
