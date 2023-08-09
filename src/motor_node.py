import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16  # You may need to define custom message types

class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        # Client to set motor velocity
        self.set_velocity_client = self.create_client(SetVelocity, 'set_velocity')

        # Client to get encoder value
        self.get_encoder_client = self.create_client(GetEncoder, 'get_encoder')

        # Publisher for the encoder data
        self.encoder_pub = self.create_publisher(Int16, 'encoder', 10)

        # Subscriber for the motor commands
        self.command_sub = self.create_subscription(
            Int16, 'command', self.command_callback, 10)

    def command_callback(self, msg):
        # Send request to set motor velocity
        request = SetVelocity.Request()
        request.velocity = msg.data
        self.set_velocity_client.call_async(request)

        # Send request to get encoder value
        request = GetEncoder.Request()
        response_future = self.get_encoder_client.call_async(request)
        response_future.add_done_callback(self.get_encoder_callback)

    def get_encoder_callback(self, response_future):
        response = response_future.result()
        encoder_msg = Int16()
        encoder_msg.data = response.encoder_value
        self.encoder_pub.publish(encoder_msg)
