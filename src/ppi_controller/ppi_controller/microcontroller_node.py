import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import Int16
from ppi_interfaces.srv import GetInt16
from ppi_interfaces.msg import Encoders

# Importing the penguinPi library
import ppi_controller.penguin_pi_lib.penguinPi as ppi

class MicrocontrollerNode(Node):
    def __init__(self):
        super().__init__('microcontroller_node')

        # Initialize PenguinPi
        ppi.init()

        # Initialize Motors
        self.mLeft = ppi.Motor('AD_MOTOR_L')
        self.mRight = ppi.Motor('AD_MOTOR_R')

        self.multi = ppi.Multi('AD_MULTI')

        # Initialize Other Components (optional)
        self.multi = ppi.Multi('AD_MULTI')
        self.voltage = ppi.AnalogIn('AD_ADC_V')
        self.current = ppi.AnalogIn('AD_ADC_C')
        self.hat = ppi.Hat('AD_HAT')
        self.led2 = ppi.LED('AD_LED_2')
        self.led3 = ppi.LED('AD_LED_3')
        self.led4 = ppi.LED('AD_LED_4')

        # Get Initial Values
        self.multi.clear_data()
        self.multi.get_encoders()
        self.mLeft.get_all()
        self.mRight.get_all()

        # Services for Motor Getters
        self.srv_get_velocity_left = self.create_service(GetInt16, '/motor/get_velocity_left', self.callback_get_velocity_left)
        self.srv_get_velocity_right = self.create_service(GetInt16, '/motor/get_velocity_right', self.callback_get_velocity_right)

        # Subscriptions for Motor Setters
        self.sub_set_velocity_left = self.create_subscription(Int16, '/motor/set_velocity_left', self.callback_set_velocity_left, 10)
        self.sub_set_velocity_right = self.create_subscription(Int16, '/motor/set_velocity_right', self.callback_set_velocity_right, 10)

        # Publishers
        self.pub_encoders = self.create_publisher(Encoders, '/motor/encoders', 10)
        timer_period = 0.005  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def callback_get_velocity_left(self, request, response):
        response.data = self.mLeft.get_velocity()
        return response

    def callback_get_velocity_right(self, request, response):
        response.data = self.mRight.get_velocity()
        return response

    def callback_set_velocity_left(self, msg):
        self.mLeft.set_velocity(msg.data)

    def callback_set_velocity_right(self, msg):
        self.mRight.set_velocity(msg.data)

    def timer_callback(self):
        # Publish encoder values
        msg = Encoders()
        encoders = self.multi.get_encoders()
        msg.left_encoder = encoders[0]
        msg.right_encoder = encoders[1]
        msg.header.stamp = self.get_clock().now().to_msg()
        # self.get_logger().info(f'Publishing: {msg.data}')

        self.pub_encoders.publish(msg)

    def on_shutdown(self):
        # Properly close the UART connection and stop motors
        ppi.close()

def main(args=None):
    rclpy.init(args=args)
    node = MicrocontrollerNode()
    rclpy.get_default_context().on_shutdown(node.on_shutdown)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()