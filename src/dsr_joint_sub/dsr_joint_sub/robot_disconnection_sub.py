import rclpy
from rclpy.node import Node
from dsr_msgs2.msg import RobotDisconnection


class RobotDisconnectionSubscriber(Node):

    def __init__(self):
        super().__init__('robot_disconnection_subscriber')

        self.subscription = self.create_subscription(
            RobotDisconnection,
            '/dsr01/robot_disconnection',
            self.disconnection_callback,
            10  # QoS depth
        )

        self.get_logger().info('Subscribed to /dsr01/robot_disconnection')

    def disconnection_callback(self, msg: RobotDisconnection):
        self.get_logger().info('--- Robot Disconnection ---')
        self.get_logger().info(f'Received: {msg}')


def main(args=None):
    rclpy.init(args=args)

    node = RobotDisconnectionSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
