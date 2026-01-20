import time

import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetRobotState

from web_ui.firebase_config import init_firebase, get_reference


class RobotStateFirebaseNode(Node):
    def __init__(self):
        super().__init__('robot_state_firebase_node')
        self.get_logger().info("Robot State Firebase Node 시작")

        init_firebase(self.get_logger())

        self.state_ref = get_reference('/robot_status/robot_state')

        self.client = self.create_client(
            GetRobotState,
            '/dsr01/system/get_robot_state'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.req = GetRobotState.Request()

        # 1초마다 로봇 상태 조회
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        future = self.client.call_async(self.req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                robot_state = response.robot_state

                # 상태 코드를 문자열로 변환
                state_names = {
                    0: 'INITIALIZING',
                    1: 'STANDBY',
                    2: 'MOVING',
                    3: 'SAFE_OFF',
                    4: 'TEACHING',
                    5: 'SAFE_STOP',
                    6: 'EMERGENCY_STOP',
                    7: 'HOMMING',
                    8: 'RECOVERY',
                    9: 'SAFE_STOP2',
                    10: 'SAFE_OFF2',
                }
                state_name = state_names.get(robot_state, 'UNKNOWN')

                self.state_ref.update({
                    'state_code': robot_state,
                    'state_name': state_name,
                    'last_update': time.time()
                })
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateFirebaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
