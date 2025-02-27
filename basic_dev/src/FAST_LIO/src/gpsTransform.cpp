#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// 全局变量
ros::Subscriber odom_sub, gps_sub;
tf::TransformBroadcaster* tf_broadcaster;

Eigen::Matrix4d odom_mat;
Eigen::Matrix4d gps_mat;
Eigen::Matrix4d mat;
tf::Transform transform;

bool odom_received = false, gps_received = false;

Eigen::Matrix3d quaternionToRotationMatrix(const geometry_msgs::Quaternion& q)
{
    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf::Matrix3x3 tf_m(tf_q);
    Eigen::Matrix3d rotation;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotation(i, j) = tf_m[i][j];
        }
    }
    return rotation;
}

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom_received = true;

    Eigen::Matrix3d rotation = quaternionToRotationMatrix(msg->pose.pose.orientation);

    odom_mat.setIdentity();
    odom_mat.block<3, 3>(0, 0) = rotation;
    odom_mat(0, 3) = msg->pose.pose.position.x;
    odom_mat(1, 3) = msg->pose.pose.position.y;
    odom_mat(2, 3) = msg->pose.pose.position.z;
}

void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!odom_received) return;
    gps_received = true;

    Eigen::Matrix3d rotation = quaternionToRotationMatrix(msg->pose.orientation);

    gps_mat.setIdentity();
    gps_mat.block<3, 3>(0, 0) = rotation;
    gps_mat(0, 3) = msg->pose.position.x;
    gps_mat(1, 3) = msg->pose.position.y;
    gps_mat(2, 3) = msg->pose.position.z;

    mat = odom_mat * gps_mat.inverse();

    rotation = mat.block<3, 3>(0, 0);
    Eigen::Vector3d translation = mat.block<3, 1>(0, 3);

    Eigen::Quaterniond quat(rotation);

    transform.setOrigin(tf::Vector3(translation(0), translation(1), translation(2)));
    tf::Quaternion q(quat.x(), quat.y(), quat.z(), quat.w()); // (qx, qy, qz, qw)
    transform.setRotation(q);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_to_odom_tf");
    ros::NodeHandle nh;
    
    odom_sub = nh.subscribe("/Odometry", 10, odometryCallback);
    gps_sub = nh.subscribe("/airsim_node/drone_1/gps", 10, gpsCallback);
    
    tf::TransformBroadcaster broadcaster;
    tf_broadcaster = &broadcaster;

    ros::Rate rate(100);
    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
      if (!gps_received) continue;

      tf_broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "gps"));
    }

    return 0;
}
