#!usr/bin/env python3

import rclpy  # Import ROS 2 Python client library
from rclpy.node import Node  # Import Node class for ROS 2 nodes
from inputs import get_gamepad  # Import function to capture gamepad events
import inputs
import math  # Import math library for calculations
import threading  # Import threading for asynchronous execution
from interfaces.msg import Controls, ControlsArray # Import custom Controls message

class PS4ControllerPublisher(Node):
    # Constants for maximum values of trigger and joystick
    MAX_TRIG_VAL = math.pow(2, 8) - 1  # Maximum trigger value for normalization (8-bit)
    MAX_JOY_VAL = math.pow(2, 15) - 1  # Maximum joystick value for normalization (15-bit)

    def __init__(self):
        # Initialize the ROS 2 node with a name
        super().__init__('ps4_controller_publisher')
        # Create a publisher for ControlsArray messages on the 'controller_data' topic
        self.publisher_ = self.create_publisher(ControlsArray, 'controller_data', 10)
        # Set a timer to call publish_controller_data every 0.1 seconds (10 Hz)
        self.timer = self.create_timer(0.1, self.publish_controller_data)

        # Initialize variables to store controller states for each button/axis
        self.joystick_L = 0.0
        self.joystick_R = 0.0
        self.bumper_L = 0.0
        self.bumper_R = 0.0
        self.trigger_L = 0.0
        self.trigger_R = 0.0

        # Start a background thread to continuously monitor controller input
        self._monitor_thread = threading.Thread(target=self._monitor_controller, args=())
        self._monitor_thread.daemon = True  # Set thread as daemon to terminate with the main program
        self._monitor_thread.start()

    def _monitor_controller(self):
        # Infinite loop to continuously capture gamepad events
        while True:
            try:
                events = get_gamepad()  # Capture events from the gamepad
                for event in events:
                    # Map each event to the appropriate controller variable, normalizing values
                    if event.code == 'ABS_Y':
                        # Normalize Left Joystick Y-axis input to range -1 to 1
                        self.joystick_L = event.state / PS4ControllerPublisher.MAX_JOY_VAL
                    elif event.code == 'ABS_RY':
                        # Normalize Right Joystick Y-axis input to range -1 to 1
                        self.joystick_R = event.state / PS4ControllerPublisher.MAX_JOY_VAL
                    elif event.code == 'ABS_Z':  # Assuming ABS_Z corresponds to L2 trigger
                        # Normalize Left Trigger input
                        self.trigger_L = event.state / PS4ControllerPublisher.MAX_TRIG_VAL
                    elif event.code == 'ABS_RZ':  # Assuming ABS_RZ corresponds to R2 trigger
                        # Normalize Right Trigger input
                        self.trigger_R = event.state / PS4ControllerPublisher.MAX_TRIG_VAL
                    elif event.code == 'BTN_TL':  # Assuming BTN_TL corresponds to L1 bumper
                        # Set Left Bumper state (binary 0 or 1)
                        self.bumper_L = event.state
                    elif event.code == 'BTN_TR':  # Assuming BTN_TR corresponds to R1 bumper
                        # Set Right Bumper state (binary 0 or 1)
                        self.bumper_R = event.state
            except inputs.UnpluggedError:
                self.get_logger().warn("No gamepad found. Please connect a gamepad.")

    def publish_controller_data(self):
        # Create a ControlsArray message
        msg = ControlsArray()

        # Create a Controls message and populate it with the current controller values
        controls_msg = Controls()
        controls_msg.joystick_left = self.joystick_L
        controls_msg.joystick_right = self.joystick_R
        controls_msg.bumper_left = self.bumper_L
        controls_msg.bumper_right = self.bumper_R
        controls_msg.trigger_left = self.trigger_L
        controls_msg.trigger_right = self.trigger_R

        # Place the Controls message in the array
        msg.controls= [controls_msg]  

        # Publish the ControlsArray message on the 'controller_data' topic
        self.publisher_.publish(msg)
        # Log the published data for debugging purposes
        self.get_logger().info(f'Publishing: {msg.inputs}')

# Main function to initialize the ROS 2 system and run the node
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS 2 Python libraries
    ps4_controller_publisher = PS4ControllerPublisher()  # Create instance of PS4ControllerPublisher node
    rclpy.spin(ps4_controller_publisher)  # Keep node running until interrupted
    ps4_controller_publisher.destroy_node()  # Clean up the node upon shutdown
    rclpy.shutdown()  # Shut down ROS 2

# Entry point of the script
if __name__ == '__main__':
    main()
