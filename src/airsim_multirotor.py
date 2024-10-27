#!/usr/bin/python3
from airsim_common import AirSimROSWrapper
import airsim
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import cv2

class AirSimROSMultirotor(AirSimROSWrapper):

    def __init__(self, control_timeout=1):

        super().__init__(vehicle_type="multirotor")
        self.control_timeout = control_timeout

        # For correct behaviour, the control commands should be published at a rate faster than control_timeout
        # Else the vehicle will hover in place
        rospy.Subscriber(f"/{self.vehicle_name}/cmd_vel_body", Twist, self.cmd_vel_body_callback)
        rospy.Subscriber(f"/{self.vehicle_name}/cmd_vel_world", Twist, self.cmd_vel_world_callback)
        rospy.Subscriber(f"/{self.vehicle_name}/move_to_gps", NavSatFix, self.move_to_gps_callback)
        rospy.Subscriber(f"/{self.vehicle_name}/move_to_position", PoseStamped, self.move_to_position_callback)
        rospy.Subscriber(f"/{self.vehicle_name}/move_to_altitude", Float32, self.move_to_altitude_callback)
        print("Subscribe: " + f"/{self.vehicle_name}/cmd_vel_body")
        print("Subscribe: " + f"/{self.vehicle_name}/cmd_vel_world")
        print("Subscribe: " + f"/{self.vehicle_name}/move_to_gps")
        print("Subscribe: " + f"/{self.vehicle_name}/move_to_position")
        print("Subscribe: " + f"/{self.vehicle_name}/move_to_altitude")

    def cmd_vel_body_callback(self, msg):
        linear_vel = msg.linear
        angular_vel = msg.angular
        try:
            self.client.moveByVelocityBodyFrameAsync(
                linear_vel.x, linear_vel.y, linear_vel.z, 
                self.control_timeout,   # timeout after which the vehicle will hover in place
                airsim.DrivetrainType.MaxDegreeOfFreedom, 
                airsim.YawMode(True, angular_vel.z),
                self.vehicle_name
            )
        except:
            pass
        
    def cmd_vel_world_callback(self, msg):
        linear_vel = msg.linear
        angular_vel = msg.angular
        try:
            self.client.moveByVelocityAsync(
                linear_vel.x, linear_vel.y, linear_vel.z, 
                self.control_timeout,   # timeout after which the vehicle will hover in place
                airsim.DrivetrainType.MaxDegreeOfFreedom, 
                airsim.YawMode(True, angular_vel.z),
                self.vehicle_name
            )
        except:
            pass
        
    def move_to_gps_callback(self, msg):
        rospy.loginfo("Moving to GPS...")
        self.client.moveToGPSAsync(
            msg.latitude, msg.longitude, msg.altitude, 
            timeout_sec=100, 
            drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, 
            vehicle_name=self.vehicle_name
        ).join()
        rospy.loginfo("Move to GPS complete.")
    
    def move_to_position_callback(self, msg):
        rospy.loginfo("Moving to position...")
        try:
            self.client.moveToPositionAsync(
                msg.position.x, msg.position.y, msg.position.z,
                velocity=5,
                drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom, 
                vehicle_name=self.vehicle_name
            ).join()
            rospy.loginfo("Move to position complete.")
        except:
            pass

    def move_to_altitude_callback(self, msg):
        rospy.loginfo("Moving to altitude...")
        try:
            self.client.moveToZAsync(
                z=msg.data, 
                velocity=5,
                vehicle_name=self.vehicle_name
            ).join()
            rospy.loginfo("Move to altitude complete.")
        except:
            pass

    def takeoff(self):
        rospy.loginfo("Taking off...")
        self.client.takeoffAsync(vehicle_name=self.vehicle_name).join()
        rospy.loginfo("Takeoff complete.")

    def land(self):
        rospy.loginfo("Landing...")
        self.client.landAsync(timeout_sec=10, vehicle_name=self.vehicle_name).join()
        rospy.loginfo("Landed.")
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.publish_sim_data()
                image = self.get_image()
                cv2.imshow("Image", image)
                cv2.waitKey(1)
            except:
                pass
            rate.sleep()
    
if __name__ == "__main__":
    print("Starting AirSim ROS Multirotor node...")
    rospy.init_node("airsim_ros_multirotor")

    airsim_ros_multirotor = AirSimROSMultirotor()
    airsim_ros_multirotor.armDisarm(True)
    airsim_ros_multirotor.takeoff()
    
    try:
        airsim_ros_multirotor.run()
    except rospy.ROSInterruptException:
        pass