#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

# Duckietown wheels command (most common)
from duckietown_msgs.msg import WheelsCmdStamped

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

class WallFollow(Node):
    def __init__(self):
        super().__init__("wall_follow")

        # ---- topics from your list ----
        self.range_topic = "/duckie02/range"
        self.wheels_topic = "/duckie02/wheels_cmd"

        # ---- choose which wall you follow ----
        self.follow_left_wall = True   # True: wall on LEFT, False: wall on RIGHT

        # ---- tuning ----
        self.target = 0.25        # meters from wall
        self.kp = 1.8             # start 1.0–3.0
        self.base = 0.25          # forward wheel command (0..1-ish)
        self.max_cmd = 0.6        # clamp for safety
        self.stop_dist = 0.12     # emergency stop distance

        self.pub = self.create_publisher(WheelsCmdStamped, self.wheels_topic, 10)
        self.sub = self.create_subscription(Range, self.range_topic, self.on_range, 10)

        self.get_logger().info(f"WallFollow running. range={self.range_topic}, wheels={self.wheels_topic}")

    def publish_wheels(self, left, right):
        msg = WheelsCmdStamped()
        msg.vel_left = float(left)
        msg.vel_right = float(right)
        self.pub.publish(msg)

    def stop(self):
        self.publish_wheels(0.0, 0.0)

    def on_range(self, msg: Range):
        d = msg.range

        # invalid / missing reading => stop
        if (not math.isfinite(d)) or d <= 0.0:
            self.stop()
            return

        # emergency stop if too close
        if d < self.stop_dist:
            self.stop()
            return

        # error > 0 means: too close (because target - d is positive)
        error = self.target - d

        # steering strength
        turn = self.kp * error

        # For LEFT wall:
        # - too close => turn RIGHT => left wheel faster, right wheel slower
        # For RIGHT wall: flip sign
        if not self.follow_left_wall:
            turn = -turn

        left = self.base + turn
        right = self.base - turn

        left = clamp(left, -self.max_cmd, self.max_cmd)
        right = clamp(right, -self.max_cmd, self.max_cmd)

        self.publish_wheels(left, right)

def main():
    rclpy.init()
    node = WallFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
