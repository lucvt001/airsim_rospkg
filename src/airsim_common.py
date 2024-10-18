#!/usr/bin/python3
import airsim
import numpy as np
import cv2
import rospy, tf
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import NavSatFix

class AirSimROSWrapper():

    def __init__(self, vehicle_type=""):

        self.vehicle_type = vehicle_type
        ip = rospy.get_param("~ip")
        port = rospy.get_param("~port")
        self.vehicle_name = rospy.get_param("~vehicle_name")

        if self.vehicle_type == "multirotor":
            self.client = airsim.MultirotorClient(ip=ip, port=port)
        elif self.vehicle_type == "car":
            self.client = airsim.CarClient(ip=ip, port=port)
        else:
            raise ValueError("Unsupported vehicle type. Use 'multirotor' or 'car'.")       
        self.client.confirmConnection()    

        self.gps_pub = rospy.Publisher(f"/{self.vehicle_name}/gps_location", NavSatFix, queue_size=1)
        self.pos_pub = rospy.Publisher(f"/{self.vehicle_name}/fused_pose", PoseStamped, queue_size=1)
        self.vel_pub = rospy.Publisher(f"/{self.vehicle_name}/fused_velocity", Twist, queue_size=1)
        print("Publish sensor_msgs/NavSatFix: " + f"/{self.vehicle_name}/gps_location")
        print("Publish geometry_msgs/PoseStamped: " + f"/{self.vehicle_name}/fused_position")
        print("Publish geometry_msgs/Twist : " + f"/{self.vehicle_name}/fused_velocity")
        self.tf_broadcaster = tf.TransformBroadcaster()

    def get_vehicle_estimated_state(self):
        if self.vehicle_type == "multirotor":
            self.kinematic_estimated = self.client.getMultirotorState(vehicle_name=self.vehicle_name).kinematics_estimated
        elif self.vehicle_type == "car":
            self.kinematic_estimated = self.client.getCarState(vehicle_name=self.vehicle_name).kinematics_estimated

    def publish_gps_location(self):
        gps_location = self.client.getGpsData(vehicle_name=self.vehicle_name).gnss.geo_point

        gps_msg = NavSatFix()
        gps_msg.latitude = gps_location.latitude
        gps_msg.longitude = gps_location.longitude
        gps_msg.altitude = gps_location.altitude

        self.gps_pub.publish(gps_msg)

    def publish_fused_pose(self):
        fused_position = self.kinematic_estimated.position
        fused_orientation = self.kinematic_estimated.orientation

        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.vehicle_name + "/base_link"

        pose_msg.pose.position.x = fused_position.x_val
        pose_msg.pose.position.y = fused_position.y_val
        pose_msg.pose.position.z = fused_position.z_val
        pose_msg.pose.orientation.x = fused_orientation.x_val
        pose_msg.pose.orientation.y = fused_orientation.y_val
        pose_msg.pose.orientation.z = fused_orientation.z_val
        pose_msg.pose.orientation.w = fused_orientation.w_val

        self.pos_pub.publish(pose_msg)

    def publish_fused_velocity(self):
        linear_velocity = self.kinematic_estimated.linear_velocity
        angular_velocity = self.kinematic_estimated.angular_velocity

        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity.x_val
        vel_msg.linear.y = linear_velocity.y_val
        vel_msg.linear.z = linear_velocity.z_val

        vel_msg.angular.x = angular_velocity.x_val
        vel_msg.angular.y = angular_velocity.y_val
        vel_msg.angular.z = angular_velocity.z_val

        self.vel_pub.publish(vel_msg)

    def publish_tf(self):
        position = self.kinematic_estimated.position
        orientation = self.kinematic_estimated.orientation

        self.tf_broadcaster.sendTransform(
            (position.x_val, position.y_val, position.z_val),
            (orientation.x_val, orientation.y_val, orientation.z_val, orientation.w_val),
            rospy.Time.now(),
            self.vehicle_name + "/base_link",
            "map"
        )

    def publish_sim_data(self):
        self.get_vehicle_estimated_state()
        self.publish_gps_location()
        self.publish_fused_pose()
        self.publish_fused_velocity()
        self.publish_tf()

    def armDisarm(self, arm: bool):
        if arm:
            self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
            self.client.armDisarm(arm, vehicle_name=self.vehicle_name)
            print(self.vehicle_name + " api-enabled and armed.")
        else:
            self.client.armDisarm(arm, vehicle_name=self.vehicle_name)
            self.client.enableApiControl(False, vehicle_name=self.vehicle_name)
            print(self.vehicle_name + " api-disabled and disarmed.")

    def get_image(self, camera_name="0", camera_type="scene"):
        '''
        camera_name: "0" or "1" for multirotor, "0" for car, or whatever you name the camera in settings.json \n
        camera_type: "depth", "segmentation", "scene", "disparity", "normals".'''
        cameraTypeMap = {
            "depth": airsim.ImageType.DepthVis,
            "segmentation": airsim.ImageType.Segmentation,
            "scene": airsim.ImageType.Scene,
            "disparity": airsim.ImageType.DisparityNormalized,
            "normals": airsim.ImageType.SurfaceNormals
            }
        responses = self.client.simGetImages([airsim.ImageRequest(camera_name, cameraTypeMap[camera_type], False, False),])
        if responses:
            response = responses[0]
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(response.height, response.width, 3)
            img_rgb = np.copy(img_rgb)      # Use np.copy() to enable resizing
            img_rgb = cv2.resize(img_rgb, (640, 480))
            return img_rgb
        return None

    def reset(self):
        print("Resetting...")
        self.client.reset()
        self.client.enableApiControl(True, vehicle_name=self.vehicle_name)
        self.client.armDisarm(True, vehicle_name=self.vehicle_name)
        print("Reset complete.")