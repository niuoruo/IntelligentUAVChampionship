import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import std_msgs.msg

def callback(msg):
    # 将 PointCloud2 消息转换为 pcl 点云格式
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
    
    # 处理后的点云数据
    processed_points = []
    
    # 获取点云的大小
    plsize = len(list(pc_data))
    pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)  # 重新读取
    # 遍历所有点
    for i, point in enumerate(pc_data):
        x, y, z = point

        # 将强度按点索引计算（归一化处理）
        intensity = int(i * 255 / plsize)  # Convert intensity to an integer (UINT8)

        # 将点加入到处理后的点云数据中
        processed_points.append([x, y, z, intensity])  # 在每个点中添加强度值

    # 创建 PointCloud2 消息的 header
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = msg.header.frame_id

    # 定义字段（包括 x, y, z 和 intensity）
    fields = [
        pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
        pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
        pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
        pc2.PointField('intensity', 12, pc2.PointField.UINT8, 1)
    ]

    # 使用 create_cloud 来处理带有强度的点云
    processed_pc = pc2.create_cloud(header, fields, processed_points)

    # 发布处理后的点云消息
    processed_pc_pub.publish(processed_pc)

def listener():
    rospy.init_node('lidar_preprocessor', anonymous=True)

    # 发布处理后的点云数据
    global processed_pc_pub
    processed_pc_pub = rospy.Publisher("/processed_lidar_points", PointCloud2, queue_size=10)
    
    # 订阅原始的 lidar 数据
    rospy.Subscriber("/airsim_node/drone_1/lidar", PointCloud2, callback)

    # 循环保持节点运行
    rospy.spin()

if __name__ == '__main__':
    listener()
