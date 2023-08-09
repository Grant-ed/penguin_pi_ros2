import rclpy
from rclpy.node import Node
from penguinPi import Motor

class UARTManagerNode(Node):
    def __init__(self):
        super().__init__('uart_manager_node')
        self.motor = Motor(address=0x00)  # Adjust the address as needed

        # Service to set motor velocity
        self.set_velocity_service = self.create_service(
            SetVelocity, 'set_velocity', self.set_velocity_callback)

        # Service to get encoder value
        self.get_encoder_service = self.create_service(
            GetEncoder, 'get_encoder', self.get_encoder_callback)

    def set_velocity_callback(self, request, response):
        self.motor.set_velocity(request.velocity)
        response.success = True
        return response

    def get_encoder_callback(self, request, response):
        response.encoder_value = self.motor.get_encoder()
        return response
