#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np

class intel_publisher_node(Node):

    def __init__(self):
        super().__init__("intel_publisher")
        self.intel_publisher_rgb = self.create_publisher(Image, "rgb_frame", 10)
        self.declare_parameter("fps", 30)
        self.fps = self.get_parameter("fps").value

        timer_period = 0.05
        self.br_rgb = CvBridge()
        try:
            self.timer_ = self.create_timer(timer_period, self.timer_callback)
            self.pipe = rs.pipeline()
            self.cfg = rs.config()
            self.cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, self.fps)
            self.pipe.start(self.cfg)
        except Exception as e:
            self.get_logger().error("Error shown here %r" % (e, ))

    def timer_callback(self):
        frames = self.pipe.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        self.intel_publisher_rgb.publish(self.br_rgb.cv2_to_imgmsg(color_image))
        self.get_logger().info("RGB Frame Published")



def main(args=None):
    rclpy.init(args=args)
    node = intel_publisher_node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
