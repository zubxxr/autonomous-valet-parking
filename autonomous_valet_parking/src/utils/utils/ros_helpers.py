import rclpy
from unique_identifier_msgs.msg import UUID
import uuid
import subprocess

def generate_uuid():
    return UUID(uuid=list(uuid.uuid4().bytes))

def build_ros2_pub(topic: str, msg_type: str, payload: str, once: bool = True) -> str:
    return f"ros2 topic pub {'--once ' if once else ''}{topic} {msg_type} '{payload}'"

def run_ros2_command(command):
    try:
        subprocess.run(command, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except subprocess.CalledProcessError as e:
        print(f"Error executing command: {e}")

def wait_until_route_complete(route_state_subscriber, timeout_sec=0.2):
    while route_state_subscriber.state != 6:
        rclpy.spin_once(route_state_subscriber, timeout_sec=timeout_sec)
