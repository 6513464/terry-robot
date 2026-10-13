import rclpy
import os
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer
from tf2_ros import TransformListener
from tf2_ros import TransformException
from rclpy.time import Time
import PyKDL
from gazebo_msgs.msg import ModelStates



class datasubscriber:
    def __init__(self):
        self.last_msg=None
        self.node=rclpy.create_node('data_subscriber')
        self.subscriber=self.node.create_subscription(PointCloud2, 'lidar_points', self.callback, qos_profile_sensor_data)
        # qos_profile_sensor_data 表示使用传感器数据的QoS配置，适用于传感器数据的订阅
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer, self.node)
        self.node.get_logger().info("start off")
        
        self.save_dir = os.path.expanduser(
            '~/dufomap/data/my_data/pcd'
        )
        os.makedirs(
            self.save_dir,
            exist_ok=True
        )
        self.frame_count=0
        self.msg_count=0

    def callback(self, msg):
        if self.last_msg is None:
            self.last_msg = msg
            return

        cloud_msg = self.last_msg
        self.last_msg = msg

    # -------------------------
    # 2. 每2帧只保存1帧
    # -------------------------
        self.msg_count += 1

        if self.msg_count % 1 != 0:
            return
        
        frame_id =cloud_msg.header.frame_id
        sec= cloud_msg.header.stamp.sec
        nanosec= cloud_msg.header.stamp.nanosec
        field_names = [field.name for field in cloud_msg.fields]
        # 取field的name的数值 field临时变量 遍历msg.fields 返回一个列表
        point_cloud= point_cloud2.read_points(cloud_msg, field_names=("x","y","z","intensity"), skip_nans=True)
        # skip_nans=True表示跳过包含NaN值的点  
        points = list(point_cloud)
        point_num = len(points)
        filename = os.path.join(
            self.save_dir,
            f"{self.frame_count:06d}.pcd"
        )
        try:
            transform=self.tf_buffer.lookup_transform(
                'odom',
                'lidar_link',
                Time.from_msg(cloud_msg.header.stamp)
            )
        except TransformException as e:
            self.node.get_logger().warn(
                f"Transform lookup failed: {e}"
                )
            return
        tx=transform.transform.translation.x
        ty=transform.transform.translation.y
        tz=transform.transform.translation.z
        qx=transform.transform.rotation.x
        qy=transform.transform.rotation.y
        qz=transform.transform.rotation.z
        qw=transform.transform.rotation.w
        rotation=PyKDL.Rotation.Quaternion(qx,qy,qz,qw)
        position=PyKDL.Vector(tx,ty,tz)
        T_odom_lidar=PyKDL.Frame(rotation,position)
        # 把 lidar_link 坐标系中的点转换到 odom 坐标系
        with open(filename, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")

            f.write("FIELDS x y z intensity\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")

            f.write(f"WIDTH {point_num}\n")
            f.write("HEIGHT 1\n")
            f.write(f"VIEWPOINT {tx} {ty} {tz} {qw} {qx} {qy} {qz}\n")
            f.write(f"POINTS {point_num}\n")
            f.write("DATA ascii\n")
            for point in points:

                x = float(point[0])
                y = float(point[1])
                z = float(point[2])
                intensity = float(point[3])
                point_lidar=PyKDL.Vector(x,y,z)
                point_odom=T_odom_lidar*point_lidar

                f.write(
                    f"{point_odom.x()} {point_odom.y()} {point_odom.z()} {intensity}\n"
                )


        self.node.get_logger().info(
            f"Saved frame {self.frame_count}: "
            f"{filename}, "
            f"points={point_num}"
        )
            

        self.frame_count += 1




def main():
    rclpy.init()


    node=datasubscriber()    
    try:
        rclpy.spin(node.node)
    except KeyboardInterrupt:
        pass
    node.node.destroy_node()
    rclpy.shutdown()
        
