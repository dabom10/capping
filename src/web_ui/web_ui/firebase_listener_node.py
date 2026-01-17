#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node

import firebase_admin
from firebase_admin import credentials
from firebase_admin import db


SERVICE_ACCOUNT_KEY_PATH = "/home/dongchanseo/Downloads/rokey-1f8f3-firebase-adminsdk-fbsvc-5b56366250.json"
DATABASE_URL = "https://rokey-1f8f3-default-rtdb.asia-southeast1.firebasedatabase.app"


class FirebaseListenerNode(Node):
    def __init__(self):
        super().__init__('firebase_db_listener_node')

        self.get_logger().info("Firebase Listener ROS2 Node 시작")

        # Firebase 초기화
        self.init_firebase()

        # Firebase listener는 블로킹이므로 별도 스레드에서 실행
        listener_thread = threading.Thread(
            target=self.start_firebase_listener,
            daemon=True
        )
        listener_thread.start()

    def init_firebase(self):
        try:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {
                'databaseURL': DATABASE_URL
            })
            self.get_logger().info("Firebase 초기화 완료")
        except ValueError:
            self.get_logger().info("Firebase 앱이 이미 초기화되어 있음")

    def listener_callback(self, event):
        """
        event.event_type: 'put', 'patch'
        event.path: 변경된 데이터 경로
        event.data: 변경된 데이터
        """
        self.get_logger().info("----- Firebase 데이터 변경 감지 -----")
        self.get_logger().info(f"이벤트 타입: {event.event_type}")
        self.get_logger().info(f"경로: {event.path}")
        self.get_logger().info(f"새 데이터: {event.data}")
        self.get_logger().info("-----------------------------------")

        # 👉 여기서 ROS2 publish / service 호출 / 상태 업데이트 가능

    def start_firebase_listener(self):
        ref = db.reference('/robot_status')
        ref.listen(self.listener_callback)


def main(args=None):
    rclpy.init(args=args)

    node = FirebaseListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Firebase Listener Node 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

