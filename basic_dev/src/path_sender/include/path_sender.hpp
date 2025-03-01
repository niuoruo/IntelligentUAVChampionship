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
#include "geometry_msgs/PoseWithCovarianceStamped.h"
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
#include <path_sender/WayPoints.h>
#endif

class PathSender
{
private:
    cv::Point3f station_[13] = {
        {0.0, 0.0, 0.0},
        {0, 0, 0},
        {143, -286, 8},
        {547, -518, 30},
        {1349, 103, 9.8},
        {1176, 424, 78},
        {783, 675, 5},
        {2, 287, 3},
        {713, 728, 14},
        {1032, 483, -59},
        {1327, -153, 35},
        {676, -539, 48},
        {263, -399, -0.2}
    };

      cv::Point3f Transit_hub_[13] = {
        {0, 0, 0},
        {650.39, 75.15, 136.33},
        {665.5, 44.8, 136.33},
        {706.9, 47, 136.33},
        {726, 88, 136.33},
        {703, 122, 136.33},
        {657, 126, 136.33},
        {650.39, 75.15, 163.9},
        {657, 126, 163.9},
        {703, 122, 163.9},
        {726, 88, 163.9},
        {706.9, 47, 163.9},
        {665.5, -44.8, 163.9}
    };
          cv::Point3f end_point_[13] = {
        {0, 0, 0},
        {-13.5671, 1.03407e-05, 2.7},
        {129.405, -287.628, 10.8939},
        {536.148, -525.205, 32.0722},
        {1363.09, 102.74, 10.20006},
        {1189.11, 424.908, 80.1408},
        {794.359, 682.403, 5.89},
        {-13.5633, 292.444, 3.84371},
        {710.375, 743.833, 14.8641},
        {1034.66, 496.756, 59.5603},
        {1340.24, -157.917, 35.3093},
        {689.856, -550.316, 49.5634},
        {260.547, -414.272, 2.46838}
    };



public:
    ros::Subscriber initial_pose_suber, end_pose_suber, gps_pose_suber; // gps数据
    void initial_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void end_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void gps_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void POintSet();
    ros::Publisher waypoint_publisher, edited_gps_publisher;
    geometry_msgs::Point station[13];
    geometry_msgs::Point Transit_hub[13];
    geometry_msgs::Point end_point[13];
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





