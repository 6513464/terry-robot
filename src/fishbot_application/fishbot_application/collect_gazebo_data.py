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
        self.node=rclpy.create_node('data_gz_subscriber')
        self.subscriber=self.node.create_subscription(PointCloud2, 'lidar_points', self.callback, qos_profile_sensor_data)
        self.gazebo_subscriber=self.node.create_subscription(ModelStates, '/gazebo/model_states', self.gazebo_callback, qos_profile_sensor_data)
        # qos_profile_sensor_data 表示使用传感器数据的QoS配置，适用于传感器数据的订阅
        self.tf_gz_buffer=Buffer()
        self.tf_gz_listener=TransformListener(self.tf_gz_buffer, self.node)
        self.latest_gz_pose=None
        self.node.get_logger().info("gazebo collection start off")
        
        self.save_dir = os.path.expanduser(
            '~/dufomap/data/my_data_gazebo/pcd'
        )
        os.makedirs(
            self.save_dir,
            exist_ok=True
        )
        self.frame_count=0
        self.msg_count=0

    def callback(self, msg):
        if self.latest_gz_pose is None:
            self.node.get_logger().warn("Waiting for Gazebo GT pose...")
            return
        frame_id =msg.header.frame_id
        sec= msg.header.stamp.sec
        nanosec= msg.header.stamp.nanosec
        field_names = [field.name for field in msg.fields]
        # 取field的name的数值 field临时变量 遍历msg.fields 返回一个列表
        point_cloud= point_cloud2.read_points(msg, field_names=("x","y","z","intensity"), skip_nans=True)
        # skip_nans=True表示跳过包含NaN值的点  
        points = list(point_cloud)
        point_num = len(points)
        filename = os.path.join(
            self.save_dir,
            f"{self.frame_count:06d}.pcd"
        )
        pose = self.latest_gz_pose
        gx = pose.position.x
        gy = pose.position.y
        gz = pose.position.z
        gxx = pose.orientation.x
        gyy = pose.orientation.y
        gzz = pose.orientation.z
        gww = pose.orientation.w
        # 获取gazebo世界中机器人的真实位姿

        rotation_gz = PyKDL.Rotation.Quaternion(gxx,gyy,gzz,gww)
        position_gz=PyKDL.Vector(gx,gy,gz)
        T_world_basefootprint = PyKDL.Frame(rotation_gz,position_gz)
        try:
            transform = self.tf_gz_buffer.lookup_transform(
                'base_footprint',
                'lidar_link',
                Time()
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
        # lidar在basefootprint下的位姿
        # 将这个位姿进行转换就能得到T_basefootprint_lidar
        # 把 lidar_link 坐标系中的点变换到 base_footprint 坐标系
        rotation=PyKDL.Rotation.Quaternion(qx,qy,qz,qw)
        position=PyKDL.Vector(tx,ty,tz)
        T_basefootprint_lidar=PyKDL.Frame(rotation,position)
        T_world_lidar=T_world_basefootprint*T_basefootprint_lidar

        tx1=T_world_lidar.p.x()
        ty1=T_world_lidar.p.y()
        tz1=T_world_lidar.p.z()    
        qx1, qy1, qz1, qw1 = T_world_lidar.M.GetQuaternion() 
        # lidarlink在world坐标系的位姿


        with open(filename, "w") as f:
            f.write("# .PCD v0.7 - Point Cloud Data file format\n")
            f.write("VERSION 0.7\n")

            f.write("FIELDS x y z intensity\n")
            f.write("SIZE 4 4 4 4\n")
            f.write("TYPE F F F F\n")
            f.write("COUNT 1 1 1 1\n")

            f.write(f"WIDTH {point_num}\n")
            f.write("HEIGHT 1\n")
            f.write(f"VIEWPOINT {tx1} {ty1} {tz1} {qw1} {qx1} {qy1} {qz1}\n")
            f.write(f"POINTS {point_num}\n")
            f.write("DATA ascii\n")
            for point in points:

                x = float(point[0])
                y = float(point[1])
                z = float(point[2])
                intensity = float(point[3])
                point_lidar=PyKDL.Vector(x,y,z)
                point_world=T_world_lidar*point_lidar

                f.write(
                    f"{point_world.x()} {point_world.y()} {point_world.z()} {intensity}\n"
                )


        self.node.get_logger().info(
            f"Saved frame {self.frame_count}: "
            f"{filename}, "
            f"points={point_num}"
        )
            

        self.frame_count += 1

        
        #-----------使用gazebo真值进行输入---------------
    def gazebo_callback(self, msg):

        if 'fishbot' not in msg.name:
            return

        index = msg.name.index('fishbot')

        self.latest_gz_pose = msg.pose[index]

    
        

    








        









def main():
    rclpy.init()


    node=datasubscriber()    
    try:
        rclpy.spin(node.node)
    except KeyboardInterrupt:
        pass
    node.node.destroy_node()
    rclpy.shutdown()
        
