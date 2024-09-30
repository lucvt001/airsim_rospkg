#!/usr/bin/python3
from airsim_common import AirSimClient
import airsim
import rospy
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

class AirSimROSMultirotor(AirSimClient):

    def __init__(self, vehicle_name="", ip="127.0.0.1", port=41451, control_timeout=1):
        super().__init__(vehicle_type="multirotor", vehicle_name=vehicle_name, ip=ip, port=port)
        self.control_timeout = control_timeout

        # For correct behaviour, the control commands should be published at a rate faster than control_timeout
        # Else the vehicle will hover in place
        rospy.Subscriber(f"/{self.vehicle_name}/cmd_vel_body", Twist, self.cmd_vel_body_callback)
        rospy.Subscriber(f"/{self.vehicle_name}/cmd_vel_world", Twist, self.cmd_vel_world_callback)
        rospy.Subscriber(f"/{self.vehicle_name}/move_to_gps", NavSatFix, self.move_to_gps_callback)
        rospy.Subscriber(f"/{self.vehicle_name}/move_to_position", Pose, self.move_to_position_callback)
        rospy.Subscriber(f"/{self.vehicle_name}/move_to_altitude", Float32, self.move_to_altitude_callback)

        self.gps_pub = rospy.Publisher(f"/{self.vehicle_name}/gps_location", NavSatFix, queue_size=1)

    def cmd_vel_body_callback(self, msg):
        linear_vel = msg.linear
        angular_vel = msg.angular
        self.client.moveByVelocityBodyFrameAsync(
            linear_vel.x, linear_vel.y, linear_vel.z, 
            self.control_timeout,   # timeout after which the vehicle will hover in place
            airsim.DrivetrainType.MaxDegreeOfFreedom, 
            airsim.YawMode(False, angular_vel.z),
            self.vehicle_name
        )
        
    def cmd_vel_world_callback(self, msg):
        linear_vel = msg.linear
        angular_vel = msg.angular
        self.client.moveByVelocityAsync(
            linear_vel.x, linear_vel.y, linear_vel.z, 
            self.control_timeout,   # timeout after which the vehicle will hover in place
            airsim.DrivetrainType.MaxDegreeOfFreedom, 
            airsim.YawMode(False, angular_vel.z),
            self.vehicle_name
        )
        
    def move_to_gps_callback(self, msg):
        self.client.moveToGPSAsync(
            msg.latitude, msg.longitude, msg.altitude, 
            timeout_sec=100, 
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, 
            vehicle_name=self.vehicle_name
        )
    
    def move_to_position_callback(self, msg):
        self.client.moveToPositionAsync(
            msg.position.x, msg.position.y, msg.position.z,
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, 
            vehicle_name=self.vehicle_name
        )

    def move_to_altitude_callback(self, msg):
        self.client.moveToZAsync(
            z=msg.data, 
            vehicle_name=self.vehicle_name
        )
        
    def move_to_position(self, x, y, z, velocity=5):
        print(f"Moving to position: x={x}, y={y}, z={z}")
        self.client.moveToPositionAsync(x, y, z, velocity).join()
        print("Reached position.")

    def takeoff(self):
        print("Taking off...")
        self.client.takeoffAsync(vehicle_name=self.vehicle_name).join()
        print("Takeoff complete.")

    def land(self):
        print("Landing...")
        self.client.landAsync(timeout_sec=10, vehicle_name=self.vehicle_name).join()
        print("Landed.")

    def get_fused_position(self):
        return self.client.getMultirotorState().kinematics_estimated.position
    
    def get_fused_velocity(self):
        return self.client.getMultirotorState().kinematics_estimated.linear_velocity
    
    def get_fused_angular_velocity(self):
        return self.client.getMultirotorState().kinematics_estimated.angular_velocity
    
if __name__ == "__main__":
    print("Starting AirSim ROS Multirotor node...")
    rospy.init_node("airsim_ros_multirotor")
    airsim_ros = AirSimROSMultirotor(vehicle_name="Drone1", ip="172.23.80.1", port=41451)
    airsim_ros.armDisarm(True)
    airsim_ros.takeoff()
    rospy.spin()