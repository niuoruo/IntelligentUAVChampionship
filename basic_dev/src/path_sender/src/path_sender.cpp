#ifndef _PATH_SENDER_CPP_
#define _PATH_SENDER_CPP_

#include <yaml-cpp/yaml.h>
#include "path_sender.hpp"
#include <filesystem>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "path_sender"); // 初始化ros 节点，命名为 basic
    ros::NodeHandle n; // 创建node控制句柄

    PathSender go(&n);

    //n.getParam("paths",paths);
    return 0;
}

std::vector<std::vector<geometry_msgs::Point>> loadPathsFromYAML(const std::string &filename)
{
    std::vector<std::vector<geometry_msgs::Point>> paths;

    try {
        YAML::Node config = YAML::LoadFile(filename);
        if (!config["paths"]) {
            ROS_ERROR("YAML file does not contain 'paths' key.");
            return paths;
        }

        for (const auto &path_node : config["paths"]) {
            std::vector<geometry_msgs::Point> path;
            for (const auto &point_node : path_node.second) {
                if (point_node.size() != 3) {
                    ROS_WARN("Invalid point format, skipping.");
                    continue;
                }

                geometry_msgs::Point point;
                point.x = point_node[0].as<double>();
                point.y = point_node[1].as<double>();
                point.z = point_node[2].as<double>();

               // std::cout << "Point: " << point.x << ", " << point.y << ", " << point.z << std::endl;

                path.push_back(point);
            }
            paths.push_back(path);
        }
    } catch (const YAML::Exception &e) {
        ROS_ERROR("Error loading YAML file: %s", e.what());
    }

    return paths;
}
void PathSender::POintSet()
{
   for (int i = 0; i < 13; ++i)
        {
            station[i].x = station_[i].x;
            station[i].y = station_[i].y;
            station[i].z = station_[i].z;

            Transit_hub[i].x = Transit_hub_[i].x;
            Transit_hub[i].y = Transit_hub_[i].y;
            Transit_hub[i].z = Transit_hub_[i].z;

            end_point[i].x = end_point_[i].x;
            end_point[i].y = end_point_[i].y;
            end_point[i].z = end_point_[i].z;
        }
}
PathSender::PathSender(ros::NodeHandle *nh)
{  

    POintSet();
    paths=loadPathsFromYAML(std::string("/home/jsj/nnn/IntelligentUAVChampionship/basic_dev/src/path_sender/config/paths.yaml"));
    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    initial_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/initial_pose", 1, std::bind(&PathSender::initial_pose_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    end_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/end_goal", 1, std::bind(&PathSender::end_pose_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    gps_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/drone_1/gps", 1, std::bind(&PathSender::gps_pose_cb, this, std::placeholders::_1));
     timer = nh->createTimer(ros::Duration(1.0),&PathSender::timeCB,this);

    waypoint_publisher = nh->advertise<path_sender::WayPoints>("/waypoints", 1);
    edited_gps_publisher = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>("/airsim_node/drone_1/edited_gps", 1);

    ros::spin();
}

PathSender::~PathSender()
{
}


void PathSender::initial_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{ 
  //std::cout<<"initial_num_get:"<<initial_num_get<<"initian_num="<<initial_num<<std::endl;
  if(!initial_num_get)
  for(int i=1;i<=12;i++)
  {
    if(abs(msg->pose.position.x -station[i].x) <5)
    {
      initial_num = i;
      initial_num_get = true;
      break;
    }
  }    

}

void PathSender::end_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
 // std::cout<<"end_num_get:"<<end_num_get<<"end_num="<<end_num<<std::endl;
  if(!end_num_get)
  for(int i=1;i<=12;i++)
  {
    if(abs(msg->pose.position.x -station[i].x) <5)
    {
      end_num = i;
      end_num_get = true;
      break;
    }
  }

}

void PathSender::gps_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::PoseWithCovarianceStamped edited_gps;
    edited_gps.header.stamp = ros::Time::now();
    edited_gps.header.frame_id = "map";
    edited_gps.pose.pose = msg->pose;

    // static tf2_ros::TransformBroadcaster br;
    // geometry_msgs::TransformStamped transformStamped;

    // transformStamped.header.stamp = ros::Time::now();
    // transformStamped.header.frame_id = "gps";
    // transformStamped.child_frame_id = "drone";
    // transformStamped.transform.translation.x = msg->pose.position.x;
    // transformStamped.transform.translation.y = msg->pose.position.y;
    // transformStamped.transform.translation.z = msg->pose.position.z;
    // transformStamped.transform.rotation = msg->pose.orientation;

    // br.sendTransform(transformStamped);

    // edited_gps.pose.pose.position.y = -edited_gps.pose.pose.position.y;
    // edited_gps.pose.pose.position.z = -edited_gps.pose.pose.position.z;

    // for (int i = 0; i < 36; ++i) {
    //   edited_gps.pose.covariance[i] = 0.0;
    // }
    // // Example covariance values, adjust as needed
    edited_gps.pose.covariance[0] = 1e-7;   // x
    edited_gps.pose.covariance[7] = 1e-7;   // y
    edited_gps.pose.covariance[14] = 1e-7;  // z
    edited_gps.pose.covariance[21] = 0.1;  // roll
    edited_gps.pose.covariance[28] = 0.1;  // pitch
    edited_gps.pose.covariance[35] = 0.1;  // yaw
    // tf2::Quaternion q;
    // tf2::fromMsg(edited_gps.pose.pose.orientation, q);
    // tf2::Matrix3x3 m(q);

    // tf2::Matrix3x3 transform(
    //   1, 0, 0,
    //   0, -1, 0,
    //   0, 0, -1
    // );

    // m = transform * m * transform;

    // tf2::Quaternion new_q;
    // m.getRotation(new_q);

    // edited_gps.pose.pose.orientation = tf2::toMsg(new_q);

    edited_gps_publisher.publish(edited_gps);
}

void PathSender::timeCB(const ros::TimerEvent& event)
{
    if(!path_get)
    {
      if(initial_num_get && end_num_get)
      {
          path= paths[initial_num-1];//将起点到第一个转运站的路径加入
          path.emplace_back(Transit_hub[initial_num]);//中枢入口
          path.emplace_back(Transit_hub[end_num]);//中枢出口
          std::vector<geometry_msgs::Point> temp_path=paths[end_num-1];
          std::reverse(temp_path.begin(),temp_path.end());//将终点到第转运站的路径倒序
          path.insert(path.end(),temp_path.begin(), temp_path.end());//将转运站到终点的路径加入
          path.emplace_back(end_point[end_num]);//终点后面一点
          geometry_msgs::Point temp_point;
          temp_point.x=end_point[end_num].x;
          temp_point.y=end_point[end_num].y;
          temp_point.z=end_point[end_num].z+150;
          path.emplace_back(temp_point);//终点后面一点向上150m
          temp_point.x=end_point[initial_num].x;
          temp_point.y=end_point[initial_num].y;
          temp_point.z=end_point[initial_num].z+150;
          path.emplace_back(temp_point);//起点后面一点向上150m
          path.emplace_back(end_point[initial_num]);//起点后面一点
          path.emplace_back(station[initial_num]);//起点后面一点
          std::cout<<"path_get"<<std::endl;

          path_get=true;
      }
    }
    else
    {
      path_sender::WayPoints path;
      path.points = this->path;
      waypoint_publisher.publish(path);
    }

}




#endif