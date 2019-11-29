import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import Byte, Bool, Float64 #for select which channel to use and for safety mode

"""Control mux node does the heavy lifting for selecting which values get sent to the hardware_interface. It also acts
as a critical safety checkpoint monitoring the sanity of all values coming from the joystick and AI models. The joystick  
inputs read from the hardware interface node are sent directly to this node for deadzone processing, tuning, and scaling. That 
means that all joystick functionality is defined here. Other input sources such as the AI model are just clipped (constrained)
for safety and moderated as to avoid out of bounds or extremely risky behavior from our sentient brain children. Trim is 
handled on the joystick and by the AI model"""

# Joystick definitions (all values -1.0 to 1.0 except buttons)
### CH1 - axes[0] steering
### CH2 - axes[1] throttle
### CH3 - button[0] (0/1) safety
### CH4 - button[1] (0/1/2) manual, manual w/ data collection, AI
### CH5 - axes[2] AI max speed
### CH6 - axes[3] set manual max speed

class ControlMux(Node):

    def __init__(self):
        super().__init__('control_mux')

        self.declare_parameter("absolute_max_speed", value=0.5)
        self.declare_parameter("deadzone", value=0.05)
        self.declare_parameter("flip_steering", value=False)
        self.declare_parameter("ai_topic", value="/ai/cmd")
        self.declare_parameter("joy_topic", value="/joy")
        self.declare_parameter("hw_interface_topic", value="/hw/cmd")
        self.get_logger().info("Parameters loaded, control mux starting...")

        self.ai_driver_sub = self.create_subscription(AckermannDriveStamped, self.get_parameter('ai_topic').value, self.ai_cb, 2)
        self.joystick_sub = self.create_subscription(Joy, self.get_parameter('joy_topic').value, self.joy_cb, 2)
        self.hw_cmd_pub = self.create_publisher(AckermannDriveStamped, self.get_parameter('hw_interface_topic').value, 2)

        self.safety_pub = self.create_publisher(Bool, 'safety_on', 10)
        self.is_autonomous_pub = self.create_publisher(Bool, 'is_autonomous', 10)
        self.is_data_collecting_pub = self.create_publisher(Bool, 'is_data_collecting', 10)
        self.ai_max_speed_pub = self.create_publisher(Float64, 'ai_max_speed', 10)
        self.manual_max_speed_pub = self.create_publisher(Float64, 'manual_max_speed', 10)


        # operational variables
        self.is_autonomous = False #determines what data stream goes to the hw_interface (autonomous/manual)
        self.is_data_collecting = False
        self.safety_on = True # controlled by CH3
        self.flip_steering = self.get_parameter("flip_steering").value
        self.absolute_max_speed = self.get_parameter("absolute_max_speed").value # Does not change during runtime
        self.deadzone = self.get_parameter('deadzone').value
        self.manual_max_speed = 0.0 # this will change based on CH6
        self.autonomous_max_speed = 0.0 # this will change based on CH5
        self.steering = 0.0
        self.throttle = 0.0
        self.last_joy_cmd = None

    # should get update at ~100Hz from hardware controller that reads joystick
    def joy_cb(self, msg):

        if self.last_joy_cmd is None:
            self.last_joy_cmd = msg

        # toggle safety mode
        if not self.last_joy_cmd.buttons[0] == msg.buttons[0]:
           self.safety_on = not self.safety_on

        # check what mode the switch is set to
        if msg.buttons[1] == 2:
            self.is_autonomous = True
            self.is_data_collecting = False
        elif msg.buttons[1] == 1:
            self.is_data_collecting = True
            self.is_autonomous = False
        else:
            self.is_data_collecting = False
            self.is_autonomous = False

        self.manual_max_speed = (msg.axes[2]+1)/2
        if self.manual_max_speed > 0.95:
            self.manual_max_speed = 1.0
        self.autonomous_max_speed = (msg.axes[3]+1)/2
        if self.autonomous_max_speed > 0.95:
            self.autonomous_max_speed = 1.0

        if self.safety_on:
            self.throttle = 0.0
        elif not self.is_autonomous: # if the car is set to manual
            throttle  = msg.axes[1]*self.absolute_max_speed*self.manual_max_speed
            if abs(throttle) < self.deadzone:
                self.throttle = 0.0
            else:
                self.throttle = throttle
            self.steering = msg.axes[0]
            if self.flip_steering:
                self.steering*=-1.0

        # if this cb does not update throttle or steering then it is using the AI's values for throttle and steering
        # send update to thruster controller
        hw_cmd = AckermannDriveStamped()
        # print(self.get_clock().now())
        # hw_cmd.header.stamp = self.get_clock().now()
        hw_cmd.drive.steering_angle = self.steering
        hw_cmd.drive.speed = self.throttle
        self.hw_cmd_pub.publish(hw_cmd)

        # send updated operational variables
        self.safety_pub.publish(Bool(data=self.safety_on))
        self.is_autonomous_pub.publish(Bool(data=self.is_autonomous))
        self.is_data_collecting_pub.publish(Bool(data=self.is_data_collecting))
        self.ai_max_speed_pub.publish(Float64(data=self.autonomous_max_speed))
        self.manual_max_speed_pub.publish(Float64(data=self.manual_max_speed))

        # update last joy
        self.last_joy_cmd = msg



    def ai_cb(self, msg):
        if not self.safety_on and self.is_autonomous:
            throttle  = msg.drive.speed*self.absolute_max_speed*self.autonomous_max_speed
            if abs(throttle) < self.deadzone:
                self.throttle = 0
            else:
                self.throttle = throttle
            self.steering = msg.drive.steering_angle

def main(args=None):
    rclpy.init(args=args)
    control_mux = ControlMux()
    rclpy.spin(control_mux)
    control_mux.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()