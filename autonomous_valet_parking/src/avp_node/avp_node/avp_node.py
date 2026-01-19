import rclpy
from std_msgs.msg import String
import argparse
from enum import Enum, auto
import time

from vehicle.state_subscribers import RouteStateSubscriber, MotionStateSubscriber, ParkingSpotSubscriber
from vehicle.command_listener import AVPCommandListener
from utils.ros_helpers import run_ros2_command, build_ros2_pub, wait_until_route_complete
from utils.dropoff_and_parking import parking_spot_locations, is_in_drop_off_zone

class VehicleState(Enum):
    # Arrival & drop-off phase
    ARRIVED_AT_LOCATION = auto()      
    AWAITING_DROPOFF_COMMAND = auto()
    NAVIGATING_TO_DROPOFF = auto()               
    ARRIVED_AT_DROPOFF = auto()          
    DROPPING_OFF_OWNER = auto()  

    # AVP start
    WAITING_FOR_AVP_START = auto()           
    AVP_STARTED = auto()           

    # Parking flow
    FINDING_PARKING_SPOT = auto()                                                  
    DRIVING_TO_PARKING_SPOT = auto()                         
    PARKED = auto()                  

    # Retrieval phase
    WAITING_FOR_RETRIEVAL = auto()               
    CALLED_FOR_RETRIEVAL = auto()                 
    DRIVING_TO_RETRIEVAL_POINT = auto()            
    PICKING_UP_OWNER = auto()    
    LEAVING_AREA = auto()    

    # Final
    AVP_COMPLETE = auto()     
                    
def main(args=None):
    parser = argparse.ArgumentParser(description="Run AVP.")
    parser.add_argument('--debug', type=str, default='false', help="Enable debug mode (true/false)")
    args, unknown = parser.parse_known_args()
    args.debug = args.debug.lower() == 'true'

    rclpy.init(args=None)

    route_state_subscriber = RouteStateSubscriber(args)
    motion_state_subscriber = MotionStateSubscriber(args)
    avp_command_listener = AVPCommandListener(route_state_subscriber, motion_state_subscriber, args)
    parking_spot_subscriber = ParkingSpotSubscriber(args)

    initial_pose_payload = (
        "{header: {frame_id: 'map'}, "
        "pose: {pose: {position: {x: -111.28318786621094, y: -80.02529907226562, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.7889132779275692, w: 0.614504548322938}}, "
        "covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}"
        "}"
    )

    set_initial_pose = build_ros2_pub("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped", initial_pose_payload)

    drop_off_payload = (
        "{header: {frame_id: 'map'}, "
        "pose: {position: {x: -97.52228546142578, y: -55.04888916015625, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: 0.11094320526941787, w: 0.9938267480826564}}}"
    )
    head_to_drop_off = build_ros2_pub("/planning/mission_planning/goal", "geometry_msgs/msg/PoseStamped", drop_off_payload)

    retrieve_payload = (
        "{header: {frame_id: 'map'}, "
        "pose: {position: {x: -105.69026947021484, y: -57.27252960205078, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: -0.9927605445395801, w: 0.12011037093222368}}}"
    )
    
    retrieve_vehicle_goal_pose = build_ros2_pub("/planning/mission_planning/goal", "geometry_msgs/msg/PoseStamped", retrieve_payload)

    leave_area_payload = (
        "{header: {frame_id: 'map'}, "
        "pose: {position: {x: -103.84678649902344, y: -106.02139282226562, z: 0.0}, "
        "orientation: {x: 0.0, y: 0.0, z: -0.6071746465964653, w: 0.7945684039341468}}}"
    )
    leave_area_goal_pose = build_ros2_pub("/planning/mission_planning/goal", "geometry_msgs/msg/PoseStamped", leave_area_payload)

    engage_autonomous_mode = build_ros2_pub("/autoware/engage", "autoware_vehicle_msgs/msg/Engage", "{engage: True}")

    current_state = VehicleState.ARRIVED_AT_LOCATION
    chosen_parking_spot = None
    first_spot_in_empty_parking_spot_list = -1

    if args.debug:
        run_ros2_command(set_initial_pose)
        
    try: 
        while rclpy.ok():
            rclpy.spin_once(avp_command_listener, timeout_sec=0.5)
            rclpy.spin_once(route_state_subscriber, timeout_sec=0.5)
            rclpy.spin_once(motion_state_subscriber, timeout_sec=0.5)
            
            if current_state == VehicleState.ARRIVED_AT_LOCATION:
                avp_command_listener.publish_vehicle_status("Arrived at location.")
                current_state = VehicleState.AWAITING_DROPOFF_COMMAND
            
            elif current_state == VehicleState.AWAITING_DROPOFF_COMMAND:
                # if "Drop-off" clicked in the AVPPanel in Autoware
                if avp_command_listener.head_to_drop_off:
                    run_ros2_command(head_to_drop_off)
                    run_ros2_command(engage_autonomous_mode)
                    current_state = VehicleState.NAVIGATING_TO_DROPOFF

            elif current_state == VehicleState.NAVIGATING_TO_DROPOFF:
                # if the vehicle is in the drop off location
                if avp_command_listener.ego_x and is_in_drop_off_zone(avp_command_listener.ego_x, avp_command_listener.ego_y):
                    current_state = VehicleState.ARRIVED_AT_DROPOFF

            elif current_state == VehicleState.ARRIVED_AT_DROPOFF:
                avp_command_listener.publish_vehicle_status("Arrived at drop-off area.")
                wait_until_route_complete(route_state_subscriber)
                current_state = VehicleState.DROPPING_OFF_OWNER

            elif current_state == VehicleState.DROPPING_OFF_OWNER:
                wait_until_route_complete(route_state_subscriber)
                avp_command_listener.handle_owner_exit()
                current_state = VehicleState.WAITING_FOR_AVP_START
                
            elif current_state == VehicleState.WAITING_FOR_AVP_START:
                if avp_command_listener.start_avp:
                    current_state = VehicleState.AVP_STARTED

            elif current_state == VehicleState.AVP_STARTED:
                avp_command_listener.publish_vehicle_status("Autonomous valet parking started...")
                time.sleep(2)
                current_state = VehicleState.FINDING_PARKING_SPOT

            elif current_state == VehicleState.FINDING_PARKING_SPOT:
                rclpy.spin_once(parking_spot_subscriber, timeout_sec=0.5)
                parking_spots = parking_spot_subscriber.available_parking_spots
                if parking_spots:
                    first_spot_in_empty_parking_spot_list = int(parking_spots.strip('[]').split(',')[0])
                    if first_spot_in_empty_parking_spot_list != chosen_parking_spot:
                        chosen_parking_spot = first_spot_in_empty_parking_spot_list

                        avp_command_listener.get_logger().info('\nAvailable parking spot found: %s' % first_spot_in_empty_parking_spot_list)
                        avp_command_listener.publish_vehicle_status("Available parking spot found.")
                        time.sleep(2)

                        current_state = VehicleState.DRIVING_TO_PARKING_SPOT
                else:
                    avp_command_listener.get_logger().warn("[WARN] No available parking spots received from the subscriber.")

            elif current_state == VehicleState.DRIVING_TO_PARKING_SPOT:
                avp_command_listener.publish_vehicle_status(f"Parking in Spot {first_spot_in_empty_parking_spot_list}.")
            
                run_ros2_command(parking_spot_locations[first_spot_in_empty_parking_spot_list])
                run_ros2_command(engage_autonomous_mode)
                route_state_subscriber.state = -1
                wait_until_route_complete(route_state_subscriber)
                current_state = VehicleState.PARKED

            elif current_state == VehicleState.PARKED:
                avp_command_listener.publish_vehicle_status("Car has been parked.")
                current_state = VehicleState.WAITING_FOR_RETRIEVAL

            elif current_state == VehicleState.WAITING_FOR_RETRIEVAL:
                if avp_command_listener.retrieve_vehicle:
                    current_state = VehicleState.CALLED_FOR_RETRIEVAL

            elif current_state == VehicleState.CALLED_FOR_RETRIEVAL:
                avp_command_listener.publish_vehicle_status("Car is called for retrieval.") 
                current_state = VehicleState.DRIVING_TO_RETRIEVAL_POINT

            elif current_state == VehicleState.DRIVING_TO_RETRIEVAL_POINT:
                run_ros2_command(retrieve_vehicle_goal_pose)
                run_ros2_command(engage_autonomous_mode)
                route_state_subscriber.state = -1
                wait_until_route_complete(route_state_subscriber)
                current_state = VehicleState.PICKING_UP_OWNER

            elif current_state == VehicleState.PICKING_UP_OWNER:
                avp_command_listener.publish_vehicle_status("Car has been retrieved.")
                time.sleep(2)
                avp_command_listener.publish_vehicle_status("Owner is getting in...")
                time.sleep(5)
                avp_command_listener.publish_vehicle_status("Owner is inside.")
                time.sleep(2)
                current_state = VehicleState.LEAVING_AREA

            elif current_state == VehicleState.LEAVING_AREA:
                avp_command_listener.publish_vehicle_status("Leaving for new destination.")
                run_ros2_command(leave_area_goal_pose)
                run_ros2_command(engage_autonomous_mode)
                route_state_subscriber.state = -1
                time.sleep(3)
                wait_until_route_complete(route_state_subscriber)
                current_state = VehicleState.AVP_COMPLETE

            elif current_state == VehicleState.AVP_COMPLETE:
                avp_command_listener.publish_vehicle_status("AVP process complete.")
                break

    except KeyboardInterrupt:
        avp_command_listener.get_logger().info("KeyboardInterrupt detected. Shutting down gracefully...")
    except Exception as e:
        avp_command_listener.get_logger().error(f"Unexpected error: {e}")
    finally:
        avp_command_listener.destroy_node()
        route_state_subscriber.destroy_node()
        motion_state_subscriber.destroy_node()
        parking_spot_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()