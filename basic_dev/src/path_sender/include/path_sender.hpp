#ifndef _PATH_SENDER_HPP_
#define _PATH_SENDER_HPP_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include "airsim_ros/Takeoff.h"
#include "airsim_ros/Reset.h"
#include "airsim_ros/Land.h"
#include "airsim_ros/GPSYaw.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include  "sensor_msgs/Imu.h"
#include <time.h>
#include <stdlib.h>
#include "Eigen/Dense"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/Point.h> // Include the necessary header for geometry_msgs
#endif

class PathSender
{
private:
    geometry_msgs::Point station[13] = {
        // geometry_msgs::Point{0.0, 0.0, 0.0},
        // geometry_msgs::Point{0, 0, 0},
        // geometry_msgs::Point {143, 286, -8},
        // geometry_msgs::Point{547, 518, -30},
        // geometry_msgs::Point {1349, -103, -9.8},
        // geometry_msgs::Point {1176, -424, -78},
        // geometry_msgs::Point {783, -675, -5},
        // geometry_msgs::Point {2, -287, -3},
        // geometry_msgs::Point {713, -728, -14},
        // geometry_msgs::Point {1032, -483, 59},
        // geometry_msgs::Point {1327, 153, -35},
        // geometry_msgs::Point{676, 539, -48},
        // geometry_msgs::Point {263, 399, 0.2}
    };

    geometry_msgs::Point Transit_hub[13] = {
        // geometry_msgs::Point {0, 0, 0},
        // geometry_msgs::Point {650.39, 75.15, 136.33},
        // geometry_msgs::Point{665.5, 44.8, 136.33},
        // geometry_msgs::Point{706.9, 47, 136.33},
        // geometry_msgs::Point {726, 88, 136.33},
        // geometry_msgs::Point {703, 122, 136.33},
        // geometry_msgs::Point {657, 126, 136.33},
        // geometry_msgs::Point {650.39, 75.15, 163.9},
        // geometry_msgs::Point {657, 126, 163.9},
        // geometry_msgs::Point {703, 122, 163.9},
        // geometry_msgs::Point {726, 88, 163.9},
        // geometry_msgs::Point {706.9, 47, 163.9},
        // geometry_msgs::Point {665.5, -44.8, 163.9}
    };
        geometry_msgs::Point end_point[13] = {
        // geometry_msgs::Point{0, 0, 0},
        // geometry_msgs::Point {0, 0, 0},
        // geometry_msgs::Point {143, 286, -8},
        // geometry_msgs::Point{547, 518, -30},
        // geometry_msgs::Point {1349, -103, -9.8},
        // geometry_msgs::Point {1176, -424, -78},
        // geometry_msgs::Point {783, -675, -5},
        // geometry_msgs::Point {2, -287, -3},
        // geometry_msgs::Point{713, -728, -14},
        // geometry_msgs::Point {1032, -483, 59},
        // geometry_msgs::Point {1327, 153, -35},
        // geometry_msgs::Point {676, 539, -48},
        // geometry_msgs::Point {263, 399, 0.2}
    };



public:
    ros::Subscriber initial_pose_suber, end_pose_suber; // gps数据
    void initial_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void end_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);

    ros::Publisher waypoint_publisher;

    bool initial_num_get = false;
    bool end_num_get = false;
    bool path_get  = false;

    int initial_num, end_num;
    std::vector<geometry_msgs::Point> path;
    PathSender(ros::NodeHandle *nh);
    void timeCB(const ros::TimerEvent& event);
    std::vector<std::vector<geometry_msgs::Point>> paths;
    ros::Timer timer;
    ~PathSender();
};





