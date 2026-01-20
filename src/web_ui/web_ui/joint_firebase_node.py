import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from web_ui.firebase_config import init_firebase, get_reference


class JointFirebaseNode(Node):
    def __init__(self):
        super().__init__('joint_firebase_node')
        self.get_logger().info("Joint Firebase Node 시작")

        init_firebase(self.get_logger())

        self.joint_ref = get_reference('/robot_status/joint_states')

        self.create_subscription(
            JointState,
            '/dsr01/joint_states',
            self.joint_state_callback,
            10
        )

    def safe_float(self, value, default=0.0):
        try:
            if value is None:
                return default
            if isinstance(value, float):
                if math.isnan(value) or math.isinf(value):
                    return default
            return float(value)
        except Exception:
            return default

    def joint_state_callback(self, msg: JointState):
        joint_data = {}

        for i, name in enumerate(msg.name):
            pos_rad = self.safe_float(msg.position[i]) if i < len(msg.position) else 0.0
            pos_deg = math.degrees(pos_rad)

            joint_data[name] = {
                'position_rad': round(pos_rad, 4),
                'position_deg': round(pos_deg, 2),
                'velocity': round(self.safe_float(msg.velocity[i]), 4) if i < len(msg.velocity) else 0.0,
                'effort': round(self.safe_float(msg.effort[i]), 4) if i < len(msg.effort) else 0.0
            }

        self.joint_ref.update({
            'joints': joint_data,
            'last_update': time.time()
        })


def main(args=None):
    rclpy.init(args=args)
    node = JointFirebaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
