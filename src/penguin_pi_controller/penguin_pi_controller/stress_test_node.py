import rclpy
from rclpy.node import Node
import timeit

from std_msgs.msg import String
from penguin_pi_interfaces.srv import GetInt16, GetEncoders

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli_left_motor = self.create_client(GetInt16, 'get_encoder_left')
        self.cli_motors = self.create_client(GetEncoders, 'get_encoders')
        while not self.cli_left_motor.wait_for_service(timeout_sec=1.0) or not self.cli_motors.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('service available')
        #timer_period = 1  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0

    def send_request(self):
        # req = GetInt16.Request()
        # future = self.cli_left_motor.call_async(req)
        req = GetEncoders.Request()
        future = self.cli_motors.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        # self.get_logger().info(f'response: {future.result().data}')
        self.get_logger().info(f'response: {future.result().uint16_encoder_values}')
        return future.result()

    def timer_callback(self):
        self.get_logger().info('starting')
        response = self.send_request()
        self.get_logger().info(f'response: {response.data}')
#        n=1
#        result = timeit.timeit(self.send_request, number=n)
#        self.get_logger().info(f"Execution time is {result / n} seconds. Estimated frequency is {n / result} Hz.")

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # msg = String()
        # for i in range(10):
        #     msg.data = 'Hello World: %d' % self.i
        #     self.publisher_.publish(msg)
        #     self.get_logger().info('Publishing: "%s"' % msg.data)
        #     self.i += 1
        # replace with timeit
        n=50
        result = timeit.timeit(self.pub_message_and_increase_index, number=n)
        self.get_logger().info(f"Execution time is {result / n} seconds. Estimated frequency is {n / result} Hz.")

    def pub_message_and_increase_index(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    # minimal_publisher = MinimalPublisher()

    # rclpy.spin(minimal_publisher)

    # # Destroy the node explicitly
    # # (optional - otherwise it will be done automatically
    # # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()

    minimal_client = MinimalClientAsync()
#    rclpy.spin(minimal_client)
    minimal_client.get_logger().info('starting')
    n=500
    result = timeit.timeit(minimal_client.send_request, number=n)
    minimal_client.get_logger().info(f"Execution time is {result / n} seconds. Estimated frequency is {n / result} Hz.")

    minimal_client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
