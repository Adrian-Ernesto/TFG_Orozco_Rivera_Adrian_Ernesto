#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf_transformations import quaternion_from_euler, euler_from_quaternion

#THIS IS A USEFUL FUNCTION TO CONVERT EULER ANGLES TO QUATERNIONS AND VICEVERSA
class AnglesConverter(Node):
    def __init__(self):
        super().__init__("angle_conversion_service_server")

        self.euler_to_quaternion_ = self.create_service(EulerToQuaternion, "euler_to_quaternion", self.eulerToQuaternionCallback)
        self.quaternion_to_euler_ = self.create_service(QuaternionToEuler, "quaternion_to_euler", self.quaternionToEulerCallback)
        
        self.get_logger().info("Angle conversion services are ready!")

    # THIS FUNCTION RECEIVES A REQUES IN EULER AND RETURNS QUATERNIONS
    def eulerToQuaternionCallback(self, req, res):
        # THE POSITION IS DECLARED BASED ON PITCH YAW AND ROLL ANGLES
        self.get_logger().info("Requested to convert euler angles roll: %f, pitch: %f, yaw: %f, into a quaternion." % (req.roll, req.pitch, req.yaw))
        # BY USING THE TF LIBRARY THE INPUT IS TRANSFORMED
        (res.x, res.y, res.z, res.w) = quaternion_from_euler(req.roll, req.pitch, req.yaw)
        # MESSAGES ARE PRINTED WITH THE RESPONSE
        self.get_logger().info("Corresponding quaternion x: %f, y: %f, z: %f, w: %f" % (res.x, res.y, res.z, res.w))
        return res
    # SIMILAR LOGIC BUT INVERSE TO ALLOW THE OTHER TRANSORMATION
    def quaternionToEulerCallback(self, req, res):
        # COMMUNICATION IS SENT WHEN A REQUEST IS RECEIVED
        self.get_logger().info("Requested to convert quaternion: %f, y: %f, z: %f, w: %f" % (req.x, req.y, req.z, req.w))
        (res.roll, res.pitch, res.yaw) = euler_from_quaternion([req.x, req.y, req.z, req.w])
        self.get_logger().info("Corresponding euler angles roll: %f, pitch: %f, yaw: %f" % (res.roll, res.pitch, res.yaw))
        return res
    
def main():
    rclpy.init()
    angles_converter = AnglesConverter()
    rclpy.spin(angles_converter)
    angles_converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

    