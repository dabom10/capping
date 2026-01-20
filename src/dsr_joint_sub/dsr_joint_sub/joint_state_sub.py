import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateSubscriber(Node):

    def __init__(self):
        super().__init__('dsr_joint_state_subscriber')

        self.subscription = self.create_subscription(
            JointState,
            '/dsr01/joint_states',
            self.joint_state_callback,
            10  # QoS depth
        )

        self.get_logger().info('Subscribed to /dsr01/joint_states')

    def joint_state_callback(self, msg: JointState):
        self.get_logger().info('--- Joint States ---')

        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else None
            velocity = msg.velocity[i] if i < len(msg.velocity) else None
            effort = msg.effort[i] if i < len(msg.effort) else None

            self.get_logger().info(
                f'{name}: pos={position:.4f}, vel={velocity:.4f}, eff={effort:.4f}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = JointStateSubscriber()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
