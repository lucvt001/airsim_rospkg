import airsim
import numpy as np
import cv2

class AirSimClient():

    def __init__(self, vehicle_type="", vehicle_name="", ip="127.0.0.1", port=41451):
        self.vehicle_type = vehicle_type
        self.vehicle_name = vehicle_name
        if self.vehicle_type == "multirotor":
            self.client = airsim.MultirotorClient(ip=ip, port=port)
        elif self.vehicle_type == "car":
            self.client = airsim.CarClient(ip=ip, port=port)
        else:
            raise ValueError("Unsupported vehicle type. Use 'multirotor' or 'car'.")       
        self.client.confirmConnection()    

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