#!/usr/bin/python3
from airsim_common import AirSimROSWrapper
import airsim
import rospy
import time

class AirSimROSCar(AirSimROSWrapper):

    def __init__(self, control_timeout=1):

        super().__init__(vehicle_type="car")
        self.control_timeout = control_timeout
    
    def run(self):
        car_controls = airsim.CarControls()
        car_controls.throttle = 1.0
        car_controls.steering = 0
        self.client.setCarControls(car_controls, vehicle_name=self.vehicle_name)
        print("Car moving forward...")
        self.initial_time = rospy.Time.now().to_sec()

        is_mission_completed = False

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_sim_data()
            rate.sleep()

            current_time = rospy.Time.now().to_sec()
            if current_time - self.initial_time > 11 and not is_mission_completed:
                car_controls.throttle = 0
                self.client.setCarControls(car_controls, vehicle_name=self.vehicle_name)
                print("Stopped.")
                is_mission_completed = True

            elif current_time - self.initial_time > 8.5:
                car_controls.steering = 0
                self.client.setCarControls(car_controls, vehicle_name=self.vehicle_name)

            elif current_time - self.initial_time > 7.3:
                car_controls.steering = 0.2
                self.client.setCarControls(car_controls, vehicle_name=self.vehicle_name)

            elif current_time - self.initial_time > 6:   # turn left after 6 seconds
                car_controls.steering = -0.3
                self.client.setCarControls(car_controls, vehicle_name=self.vehicle_name)        
        

if __name__ == "__main__":

    print("Starting AirSim ROS Car node...")
    rospy.init_node("airsim_ros_car")

    airsim_ros_car = AirSimROSCar()
    airsim_ros_car.armDisarm(True)
    time.sleep(4)
    t = time.time()
    while time.time() - t < 7:
        airsim_ros_car.publish_sim_data()
        rospy.sleep(0.1)

    try:
        airsim_ros_car.run()
    except rospy.ROSInterruptException:
        pass
