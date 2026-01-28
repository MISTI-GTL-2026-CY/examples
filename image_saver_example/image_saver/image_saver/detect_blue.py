#!/usr/bin/python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
from cv_bridge import CvBridge


class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.output_dir = "/workspace/images/"
        os.makedirs(self.output_dir, exist_ok=True)
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        if self.vehicle_name is None:
            self.get_logger().error("VEHICLE_NAME not set")
            return
        self.counter = 0
        self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.callback, 10)

    def callback(self, msg):
        # Convert compressed image to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Blue mask
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        #debugging
        cv2.imshow("Blue Mask", mask)
        cv2.waitKey(1)

        # How much blue is present?
        blue_pixels = cv2.countNonZero(mask)

        if blue_pixels > 5000:  # threshold (tune this)
            self.get_logger().info("BLUE OBJECT DETECTED")

            # Optional: save only when blue is detected
            filename = os.path.join(self.output_dir, f"blue_{self.counter}.jpg")
            cv2.imwrite(filename, frame)
            self.counter += 1


def main():
    rclpy.init()
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
