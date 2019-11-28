import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Byte, Bool #for select which channel to use and for safety mode

#my_node.get_clock().now()

"""Control mux node does the heavy lifting for selecting which values get sent to the hardware_interface. It also acts
as a critical safety checkpoint monitoring the sanity of all values coming from the joystick and AI models. The joystick  
inputs read from the hardware interface node are sent directly to this node for deadzone processing, tuning, and scaling. That 
means that all joystick functionality is defined here. Other input sources such as the AI model are just clipped (constrained)
for safety and moderated as to avoid out of bounds or extremely risky behavior from our sentient brain children. Trim is 
handled on the joystick and by the AI model"""

# Joystick definitions (all values -1.0 to 1.0 except buttons)
### CH1 - axes[0] steering
### CH2 - axes[1] throttle
### CH3 - button[0] (0/1) hold constant speed button (lock speed)
### CH4 - button[1] (0/1/2) manual, manual w/ data collection, AI
### CH5 - axes[2] AI max speed
### CH6 - axes[3] set manual max speed and adjust locked speed

class ControlMux(Node):

    def __init__(self):
        super().__init__('control_mux')
        # self.get_logger().info('I heard: "%s"' % self.get_parameter('absolute_max_speed').value)
        self.declare_parameter("absolute_max_speed")
        self.declare_parameter("ai_topic")
        self.declare_parameter("joy_topic")
        self.declare_parameter("hw_interface_topic")
        print(self.get_parameter('hw_interface_topic')._value)

def main(args=None):
    rclpy.init(args=args)
    control_mux = ControlMux()
    rclpy.spin(control_mux)
    control_mux.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()