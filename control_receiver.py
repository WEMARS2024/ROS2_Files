#! usr/bin/env python3
import rclpy 
import serial
from rclpy.node import Node
from controller_interfaces.msg import ControlsArray
from controller_interfaces.msg import Controls



class Control_Receiver_Node(Node):

    def __init__(self):
        super().__init__("Controls_Receiver")
        self.subscriber_ = self.create_subscription(ControlsArray, "motor_controls", self.set_motor_values, 10)
        self.get_logger().info("Receiver Node has started")
        self.acc_function = self.declare_parameter("Acceleration", 2).value
        self.current_value = [0.00, 0.00]


    def set_motor_values(self, msg):
        value = Controls()
        value = msg.motor_controls[0]

        mr = value.motor_right

        if value.motor_left > 0:
            formatted_left = -abs(value.motor_left)
        else:
            formatted_left = abs(value.motor_left)

        target_value = [formatted_left, mr]
        step_size = 0.01

        #self.get_logger().info("Motor Values: " + str(target_value[0]) + ", " + str(target_value[1]))

        self.gradual_increase(target_value, step_size)



    def gradual_increase(self, target_value, step_size, min_step = 0.01):
        diff_left = abs(target_value[0] - self.current_value[0])
        diff_right = abs(target_value[1] - self.current_value[1]) 


        step_left = (step_size * (max((1 - diff_left)**(self.acc_function), min_step)))*20 #change based on speed of gradual increase
        step_right = (step_size * (max((1 - diff_right)**(self.acc_function), min_step)))*20 #change based on speed of gradual increase

        step_value = [step_left, step_right]


        self.get_logger().info("Currents: " + str(self.current_value[0]) + ", " + str(self.current_value[1]))
        self.get_logger().info("Targets: " + str(target_value[0]) + ", " + str(target_value[1]))
        self.get_logger().info(str(step_left) + ", " + str(step_right))

        self.update_motor_value(self.current_value, target_value, step_value)


    def update_motor_value(self, current_value, target_value, step_value):

        for i in range(len(target_value)):

            if target_value[i] > current_value[i]:
                current_value[i] += step_value[i]

                if current_value[i] > target_value[i]:
                    current_value[i] = target_value[i]

            elif target_value[i] < current_value[i]:
                current_value[i] -= step_value[i]

                if current_value[i] < target_value[i]:
                    current_value[i] = target_value[i]

        return current_value


    def send_to_canhub(self):
        pass
      

def main(args=None):
    rclpy.init(args=args)
    node = Control_Receiver_Node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
