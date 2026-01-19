import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from utils.dropoff_and_parking import is_in_drop_off_zone
import time

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1
)

class AVPCommandListener(Node):
    def __init__(self, route_state_subscriber, motion_state_subscriber, args):
        super().__init__('avp_command_listener')

        self.cars_in_zone = set()

        self.ego_x = None
        self.ego_y = None

        self.route_state_subscriber = route_state_subscriber
        self.motion_state_subscriber = motion_state_subscriber

        # States
        self.head_to_drop_off = False
        self.start_avp = False
        self.retrieve_vehicle = False
        self.status_initialized = False
        self.initiate_parking = False

        # Publishers
        self.local_status_publisher = self.create_publisher(String, '/avp/status', 10)

        # Subscribers   
        self.receive_button_command_sub = self.create_subscription(String, '/avp/command', self.command_callback, 10)
        self.is_in_drop_off_sub = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)

    def publish_vehicle_status(self, status: str):
        # Publish local status message
        self.local_status_publisher.publish(String(data=status))

    def odom_callback(self, msg):
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y   

    ## When a button in the AVPPanel is clicked, it sends different messages
    def command_callback(self, msg):
        if msg.data == "head_to_dropoff":
            self.head_to_drop_off = True
            self.publish_vehicle_status("Heading to drop-off zone")
        elif msg.data == "start_avp":
            self.start_avp = True
        elif msg.data == "retrieve":
            self.retrieve_vehicle = True

    def handle_owner_exit(self):
        self.publish_vehicle_status("Owner is exiting...")
        time.sleep(5)
        self.publish_vehicle_status("Owner has exited.")
        time.sleep(2)
        
        rclpy.spin_once(self.route_state_subscriber, timeout_sec=0.1)
        rclpy.spin_once(self.motion_state_subscriber, timeout_sec=0.1)

        if self.route_state_subscriber.state == 6:
            self.get_logger().info(f"[DEBUG] Reached state 6 after spin_once.")
            self.publish_vehicle_status("On standby...")
            return True
        
        # Keep retrying until route state == 6
        while self.route_state_subscriber.state != 6:
            rclpy.spin_once(self.route_state_subscriber, timeout_sec=0.2)

