#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState


class MoveObjects(Node):

    def __init__(self):
        super().__init__('move_dynamic_objects')

        self.client = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'Waiting for /gazebo/set_entity_state...'
            )

        self.start_time = self.get_clock().now()

        # 20 Hz 更新
        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.get_logger().info('Dynamic object controller started.')


    def set_pose(self, name, x, y, z, yaw=0.0):

        request = SetEntityState.Request()

        state = EntityState()

        state.name = name

        state.pose.position.x = float(x)
        state.pose.position.y = float(y)
        state.pose.position.z = float(z)

        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)

        state.reference_frame = 'world'

        request.state = state

        self.client.call_async(request)


    def timer_callback(self):

        now = self.get_clock().now()

        t = (
            now.nanoseconds -
            self.start_time.nanoseconds
        ) / 1e9

        # ==================================
        # 汽车
        #
        # x 在 -3 ～ +3 m 之间往返
        # y 固定 2.5 m
        # ==================================

        car_x = 0.5 * math.sin(0.25 * t)

        # 根据运动方向改变车头
        velocity_sign = math.cos(0.25 * t)

        if velocity_sign >= 0:
            car_yaw = 0.0
        else:
            car_yaw = math.pi

        self.set_pose(
            name='dynamic_car',
            x=car_x,
            y=2.5,
            z=0.75,
            yaw=car_yaw
        )


        # ==================================
        # 行人
        #
        # y 在 -1 ～ +1 m 之间往返
        # x 固定 4 m
        # ==================================

        person_y = 1.0 * math.sin(0.5 * t)

        velocity_sign = math.cos(0.5 * t)

        if velocity_sign >= 0:
            person_yaw = math.pi / 2.0
        else:
            person_yaw = -math.pi / 2.0

        self.set_pose(
            name='dynamic_person',
            x=4.0,
            y=person_y,
            z=0.9,
            yaw=person_yaw
        )


def main(args=None):

    rclpy.init(args=args)

    node = MoveObjects()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
