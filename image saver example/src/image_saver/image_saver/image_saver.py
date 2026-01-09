#!/usr/bin/python3

import os
import rclpy
from rclpy.node import node
from sensor_msgs.msg import CompressedImage

class imageSaver(node):
    def __init__(self):
        super().__init__(self, node_name='image_saver')
        self.output_dir = "/workspace/images/"
        os.makedirs(self.output_dir, exist_ok=True)
        self.vehicle_name = os.getenv('VEHICLE_NAME')
        self.counter = 0
        self.create_subscription(CompressedImage, f'/{self.vehicle_name}/image/compressed', self.save_image, 10)

    def save_image(self, msg):
        if self.counter % 30 == 0:
            self.counter+=1
            return
        with open(self.output_dir+ str(self.counter)+',jpg', 'wb') as f:
            f.write(msg.data)
        self.counter+=1

def main():
    rclpy.init()
    imagesaver = imageSaver()
    rclpy.spin(imagesaver)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
