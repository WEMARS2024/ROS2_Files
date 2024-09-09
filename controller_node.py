#! usr/bin/env python3
import rclpy 
from rclpy.node import Node
from dualsense_controller import DualSenseController
from controller_interfaces.msg import ControlsArray
from controller_interfaces.msg import Controls



class Controller_Node(Node):

    def __init__(self):
        super().__init__("Controller_Node")
        self.publisher_ = self.create_publisher(ControlsArray, "motor_controls", 10)

        self.controller = DualSenseController()
        self.controller.activate()
        self.controller.on_error(self.on_error)

        device_infos = DualSenseController.enumerate_devices()
        if len(device_infos) < 1:
            raise Exception('No Controller Connected.')
        
        self.timer_ = self.create_timer(0.1, self.control_publisher)

    
    def control_publisher(self):
        current_value = ControlsArray()
        message = Controls()
        
        left_value = self.controller.left_stick_y._get_value()
        right_value = self.controller.right_stick_y._get_value()
        message.motor_left = left_value
        message.motor_right = right_value

        current_value.motor_controls.append(message)

        self.publisher_.publish(current_value)
        self.get_logger().info("Published current joystick values")

    def on_error(self, error):
        self.get_logger().error("an error occured %r" % (error, ))



def main(args=None):
    rclpy.init(args=args)
    node = Controller_Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
