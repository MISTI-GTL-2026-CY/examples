#!/usr/bin/python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Range
from std_msgs.msg import Header
from duckietown_msgs.msg import WheelsCmdStamped

import cv2
import numpy as np

# ======================
# Constants
# ======================
DEFAULT_VEL_LEFT = 0.475
DEFAULT_VEL_RIGHT = 0.5
OBSTACLE_DISTANCE = 0.2


class VisionController(Node):
    def __init__(self):
        super().__init__('vision_controller')

        self.vehicle_name = os.getenv('VEHICLE_NAME')
        if not self.vehicle_name:
            self.get_logger().error('VEHICLE_NAME not set')
            return

        # Publishers
        self.wheels_pub = self.create_publisher(
            WheelsCmdStamped,
            f'/{self.vehicle_name}/wheels_cmd',
            10
        )

        # Subscriptions
        self.create_subscription(
            CompressedImage,
            f'/{self.vehicle_name}/image/compressed',
            self.image_callback,
            10
        )

        self.create_subscription(
            Range,
            f'/{self.vehicle_name}/range',
            self.range_callback,
            10
        )

        # State
        self.obstacle_detected = False

    # ======================
    # Range sensor callback
    # ======================
    def range_callback(self, msg: Range):
        self.obstacle_detected = msg.range < OBSTACLE_DISTANCE

    # ======================
    # Image callback
    # ======================
    def image_callback(self, msg: CompressedImage):
        if self.obstacle_detected:
            self.publish_wheels(0.0, 0.0, 'stop')
            return

        # Decode compressed image → OpenCV
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img is None:
            return

        steering = self.compute_steering(img)
        self.apply_steering(steering)

    # ======================
    # Vision processing
    # ======================
    def compute_steering(self, img):
        """Returns steering value in range [-1, 1]"""

        h, w, _ = img.shape

        # Convert to grayscale and threshold (road assumed dark)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

        # Focus on bottom half of image
        roi = mask[h // 2 :, :]

        ys, xs = np.where(roi > 0)
        if len(xs) == 0:
            return 0.0

        avg_x = np.mean(xs)
        center_x = w / 2

        # Normalized steering error
        steering = (avg_x - center_x) / center_x
        return np.clip(steering, -1.0, 1.0)

    # ======================
    # Control logic
    # ======================
    def apply_steering(self, steering):
        gain = 0.15
        left = DEFAULT_VEL_LEFT - gain * steering
        right = DEFAULT_VEL_RIGHT + gain * steering
        self.publish_wheels(left, right, 'vision_control')

    def publish_wheels(self, left, right, frame_id):
        msg = WheelsCmdStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.vel_left = float(left)
        msg.vel_right = float(right)
        self.wheels_pub.publish(msg)


# ======================
# Main
# ======================

def main():
    rclpy.init()
    node = VisionController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

