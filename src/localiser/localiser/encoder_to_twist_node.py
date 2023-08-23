import math
import rclpy
from rclpy.node import Node

from ppi_interfaces.msg import Encoders
from geometry_msgs.msg import Twist


class EncoderTranslator(Node):
    WHEEL_SEP = 0.156 # lateral wheel separation (m)
    WHEEL_DIAM = 0.065 # wheel diameter (m)
    ENCODER_RES = math.pi * WHEEL_DIAM / 384 # encoder resolution (m)

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Subscriptions
        self.subscription = self.create_subscription(Encoders, '/motor/encoders', self.encoder_callback, 10)
        self.subscription  # prevent unused variable warning

        # Publishers
        self.publisher = self.create_publisher(Twist, '/motor/twist', 10)
        self.initialised = False
    
    
    def encoder_callback(self, msg):
        '''
        This function is called every time the encoder topic is updated.
        '''
        new_left = msg.left_encoder
        new_right = msg.right_encoder
        (sec, nanosec) = msg.header.stamp
        new_time = sec + nanosec * 1e-9
        
        self.get_logger().info(f'Left: {new_left}, Right: {new_right}, Time: {new_time}')

        # Initialise the encoder values
        if not self.initialised:
            self.initialised = True
            self.left_encoder = new_left
            self.right_encoder = new_right
            return
        
        # function to calculate the difference between two encoder values (taking into account overflow)
        def encoder_difference(new_val, prev_val):
            d = new_val - prev_val
            if d > 32000:
                d = 0x10000 - d
            elif d < -32000:
                d += 0x10000
            return d

        # compute the difference since last sample and handle 16-bit wrapping
        dL = encoder_difference(new_left, self.left_encoder)
        dR = encoder_difference(new_right, self.right_encoder)

        # compute the linear and angular velocities
        vL = dL * self.ENCODER_RES / (new_time - self.prev_time)
        vR = dR * self.ENCODER_RES / (new_time - self.prev_time)
        v = (vL + vR) / 2
        w = (vR - vL) / self.WHEEL_SEP

        # update the previous values
        self.left_encoder = new_left
        self.right_encoder = new_right
        self.prev_time = new_time

        # publish the twist message
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    encoder_translator = EncoderTranslator()

    rclpy.spin(encoder_translator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    encoder_translator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()