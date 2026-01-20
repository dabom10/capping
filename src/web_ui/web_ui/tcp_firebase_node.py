import time

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetCurrentPosx

from web_ui.firebase_config import init_firebase, get_reference


class TcpFirebaseNode(Node):
    def __init__(self):
        super().__init__('tcp_firebase_node')
        self.get_logger().info("TCP Firebase Node 시작")

        init_firebase(self.get_logger())

        self.tcp_ref = get_reference('/robot_status/tcp')

        self.client = self.create_client(
            GetCurrentPosx,
            '/dsr01/aux_control/get_current_posx'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for get_current_posx service...')

        self.req = GetCurrentPosx.Request()
        self.req.ref = 0  # DR_BASE 좌표계

        # 0.5초마다 TCP 위치 조회
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        future = self.client.call_async(self.req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response is not None and response.success:
                # task_pos_info[0].data = [x, y, z, rx, ry, rz, solution_space]
                pos_data = response.task_pos_info[0].data

                tcp_data = {
                    'x': round(pos_data[0], 2),
                    'y': round(pos_data[1], 2),
                    'z': round(pos_data[2], 2),
                    'rx': round(pos_data[3], 2),
                    'ry': round(pos_data[4], 2),
                    'rz': round(pos_data[5], 2),
                    'last_update': time.time()
                }

                self.tcp_ref.update(tcp_data)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TcpFirebaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
