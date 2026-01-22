#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
로봇 공정 통합 Launch 파일
- 캡핑 로봇 노드와 쉐이킹 로봇 노드를 함께 실행
- 캡핑 로봇이 먼저 실행된 후 쉐이킹 로봇이 실행됨
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    """Launch 설명 생성"""

    # 1. 캡핑 로봇 노드
    capping_node = Node(
        package='robot',
        executable='capping_robot',
        name='capping_robot_node',
        namespace='dsr01',
        output='screen',
        emulate_tty=True,
    )

    # 2. 쉐이킹 로봇 노드 (캡핑 노드 시작 2초 후 실행)
    shaking_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot',
                executable='shaking_robot',
                name='shaking_robot_node',
                namespace='dsr01',
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    return LaunchDescription([
        capping_node,
        shaking_node,
    ])
