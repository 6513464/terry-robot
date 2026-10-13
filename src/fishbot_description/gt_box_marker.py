#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import PyKDL

from gazebo_msgs.msg import ModelStates

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import Detection3D

from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformException


class GTBoxMarker(Node):

    def __init__(self):
        super().__init__('gt_box_marker')

        # ========================================================
        # Gazebo model states
        # ========================================================
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        # ========================================================
        # 给动态目标去除算法使用
        # ========================================================
        self.detection_publisher = self.create_publisher(
            Detection3DArray,
            '/detected_objects',
            10
        )

        # ========================================================
        # 给 RViz 显示
        # ========================================================
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/gt_boxes',
            10
        )

        # ========================================================
        # TF2
        # ========================================================
        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ========================================================
        # Gazebo 机器人 model 名
        # ========================================================
        self.robot_name = 'fishbot'

        # ========================================================
        # ROS frame
        # ========================================================
        self.base_frame = 'base_link'

        # 你的点云坐标系
        self.lidar_frame = 'lidar_link'

        # ========================================================
        # 动态目标
        # ========================================================
        self.objects = {

            'dynamic_car': {
                'size': [0.8, 0.8, 1.5],
                'z_offset': 0.0,
                'color': [1.0, 0.0, 0.0]
            },

            'dynamic_person': {
                'size': [0.6, 0.6, 1.8],
                'z_offset': 0.0,
                'color': [0.0, 1.0, 0.0]
            }
        }

        self.get_logger().info(
            'GT Box started: '
            'Gazebo world -> base_link -> lidar_link'
        )

    # ============================================================
    # geometry_msgs/Pose
    #       ↓
    # PyKDL.Frame
    #
    # PyKDL.Frame 同时包含：
    #   position
    #   orientation
    # ============================================================
    def pose_to_kdl(self, pose):

        rotation = PyKDL.Rotation.Quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )

        position = PyKDL.Vector(
            pose.position.x,
            pose.position.y,
            pose.position.z
        )

        return PyKDL.Frame(
            rotation,
            position
        )

    # ============================================================
    # geometry_msgs/Transform
    #       ↓
    # PyKDL.Frame
    # ============================================================
    def transform_to_kdl(self, transform):

        rotation = PyKDL.Rotation.Quaternion(
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        )

        position = PyKDL.Vector(
            transform.translation.x,
            transform.translation.y,
            transform.translation.z
        )

        return PyKDL.Frame(
            rotation,
            position
        )

    # ============================================================
    # Gazebo callback
    # ============================================================
    def model_states_callback(self, msg):

        # ========================================================
        # 找机器人 fishbot
        # ========================================================
        if self.robot_name not in msg.name:

            self.get_logger().warn(
                f'Robot {self.robot_name} not found'
            )

            return

        robot_index = msg.name.index(
            self.robot_name
        )

        robot_pose = msg.pose[
            robot_index
        ]

        # ========================================================
        # Gazebo 给的是：
        #
        # fishbot 在 world 下的 Pose
        #
        # 直接交给 PyKDL
        # ========================================================
        T_world_base = self.pose_to_kdl(
            robot_pose
        )

        # ========================================================
        # 获取 ROS TF：
        #
        # base_link -> lidar_link
        #
        # lookup_transform(
        #     target,
        #     source,
        #     time
        # )
        #
        # 得到的 transform 可以把
        # base_link 下的数据变成 lidar_ link 下
        # ========================================================
        try:

            tf_msg = self.tf_buffer.lookup_transform(
                self.lidar_frame,
                self.base_frame,
                Time()
            )

        except TransformException as ex:

            self.get_logger().warn(
                f'Cannot get TF '
                f'{self.base_frame} -> '
                f'{self.lidar_frame}: {ex}'
            )

            return

        # ========================================================
        # TF 消息交给 PyKDL
        # ========================================================
        T_lidar_base = self.transform_to_kdl(
            tf_msg.transform
        )

        # ========================================================
        # 发布消息
        # ========================================================
        now = self.get_clock().now().to_msg()

        marker_array = MarkerArray()

        detection_array = Detection3DArray()

        detection_array.header.stamp = now
        detection_array.header.frame_id = \
            self.lidar_frame

        marker_id = 0

        # ========================================================
        # car / person
        # ========================================================
        for object_name, info in self.objects.items():

            if object_name not in msg.name:
                continue

            object_index = msg.name.index(
                object_name
            )

            object_pose = msg.pose[
                object_index
            ]

            # ====================================================
            # Gazebo object Pose
            # 直接转 PyKDL.Frame
            # ====================================================
            T_world_object = self.pose_to_kdl(
                object_pose
            )

            # ====================================================
            # world -> base_link
            #
            # 不再自己：
            # - 减坐标
            # - 转旋转矩阵
            # - 求 quaternion inverse
            # - quaternion multiply
            #
            # 全部交给 PyKDL
            # ====================================================
            T_base_object = \
                T_world_base.Inverse() * \
                T_world_object

            # ====================================================
            # base_link -> lidar_  link
            #
            # 继续直接做 Frame 运算
            # ====================================================
            T_lidar_object = \
                T_lidar_base * \
                T_base_object

            # ====================================================
            # 可选 box 中心 z 修正
            # ====================================================
            if info['z_offset'] != 0.0:

                offset = PyKDL.Frame(
                    PyKDL.Rotation.Identity(),
                    PyKDL.Vector(
                        0.0,
                        0.0,
                        info['z_offset']
                    )
                )

                T_lidar_object = \
                    T_lidar_object * offset

            # ====================================================
            # 从 PyKDL.Frame 直接拿位置
            # ====================================================
            x = T_lidar_object.p.x()
            y = T_lidar_object.p.y()
            z = T_lidar_object.p.z()

            # ====================================================
            # 从 PyKDL.Frame 直接拿 quaternion
            #
            # 返回：
            # qx, qy, qz, qw
            # ====================================================
            qx, qy, qz, qw = \
                T_lidar_object.M.GetQuaternion()

            # ====================================================
            #
            # RViz Marker
            #
            # ====================================================
            marker = Marker()

            marker.header.frame_id = \
                self.lidar_frame

            marker.header.stamp = now

            marker.ns = 'gt_boxes'
            marker.id = marker_id

            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            # ----------------------------------------------------
            # position
            # ----------------------------------------------------
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(z)

            # ----------------------------------------------------
            # orientation
            # ----------------------------------------------------
            marker.pose.orientation.x = float(qx)
            marker.pose.orientation.y = float(qy)
            marker.pose.orientation.z = float(qz)
            marker.pose.orientation.w = float(qw)

            # ----------------------------------------------------
            # size
            # ----------------------------------------------------
            marker.scale.x = \
                float(info['size'][0])

            marker.scale.y = \
                float(info['size'][1])

            marker.scale.z = \
                float(info['size'][2])

            # ----------------------------------------------------
            # color
            # ----------------------------------------------------
            marker.color.r = \
                float(info['color'][0])

            marker.color.g = \
                float(info['color'][1])

            marker.color.b = \
                float(info['color'][2])

            marker.color.a = 0.45

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = \
                300000000

            marker_array.markers.append(
                marker
            )

            # ====================================================
            #
            # Detection3D
            #
            # ====================================================
            detection = Detection3D()

            detection.header.stamp = now

            detection.header.frame_id = \
                self.lidar_frame

            # ----------------------------------------------------
            # Bounding box center position
            # ----------------------------------------------------
            detection.bbox.center.position.x = \
                float(x)

            detection.bbox.center.position.y = \
                float(y)

            detection.bbox.center.position.z = \
                float(z)

            # ----------------------------------------------------
            # Bounding box orientation
            # ----------------------------------------------------
            detection.bbox.center.orientation.x = \
                float(qx)

            detection.bbox.center.orientation.y = \
                float(qy)

            detection.bbox.center.orientation.z = \
                float(qz)

            detection.bbox.center.orientation.w = \
                float(qw)

            # ----------------------------------------------------
            # Bounding box size
            # ----------------------------------------------------
            detection.bbox.size.x = \
                float(info['size'][0])

            detection.bbox.size.y = \
                float(info['size'][1])

            detection.bbox.size.z = \
                float(info['size'][2])

            # ----------------------------------------------------
            # 放到 Detection3DArray
            # ----------------------------------------------------
            detection_array.detections.append(
                detection
            )

            marker_id += 1

        # ========================================================
        # 发布 RViz box
        # ========================================================
        self.marker_publisher.publish(
            marker_array
        )

        # ========================================================
        # 发布检测框
        #
        # /detected_objects
        # frame = lidarlink
        # ========================================================
        self.detection_publisher.publish(
            detection_array
        )


def main(args=None):

    rclpy.init(args=args)

    node = GTBoxMarker()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':

    main()
