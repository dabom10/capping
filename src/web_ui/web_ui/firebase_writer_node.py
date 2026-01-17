#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

import firebase_admin
from firebase_admin import credentials
from firebase_admin import db


SERVICE_ACCOUNT_KEY_PATH = "/home/dongchanseo/Downloads/rokey-1f8f3-firebase-adminsdk-fbsvc-5b56366250.json"
DATABASE_URL = "https://rokey-1f8f3-default-rtdb.asia-southeast1.firebasedatabase.app"


class FirebaseWriterNode(Node):
    def __init__(self):
        super().__init__('firebase_db_writer_node')

        self.get_logger().info("Firebase Writer ROS2 Node 시작")

        # Firebase 초기화
        self.init_firebase()

        self.ref = db.reference('/robot_status')

        self.job_count = 0

        # ROS2 타이머 (1초 주기)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def init_firebase(self):
        try:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {
                'databaseURL': DATABASE_URL
            })
            self.get_logger().info("Firebase 초기화 완료")
        except ValueError:
            self.get_logger().info("Firebase 앱이 이미 초기화되어 있음")

    def timer_callback(self):
        self.job_count += 1

        self.ref.update({
            'completed_jobs': self.job_count,
            'last_update_timestamp': time.time()
        })

        self.get_logger().info(
            f"Firebase DB 업데이트: completed_jobs={self.job_count}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = FirebaseWriterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Firebase Writer Node 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

