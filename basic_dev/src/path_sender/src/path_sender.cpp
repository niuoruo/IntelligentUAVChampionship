#ifndef _BASIC_DEV_CPP_
#define _BASIC_DEV_CPP_

#include "path_sender.hpp"

int main(int argc, char** argv)
{

    ros::NodeHandle n; // 创建node控制句柄
    ros::init(argc, argv, "basic_dev"); // 初始化ros 节点，命名为 basic
    PathSender go(&n);

    //n.getParam("paths",paths);
    return 0;
}

PathSender::PathSender(ros::NodeHandle *nh)
{  
    
    
    

    //无人机信息通过如下命令订阅，当收到消息时自动回调对应的函数
    initial_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/initial_pose", 1, std::bind(&PathSender::initial_pose_cb, this, std::placeholders::_1));//状态真值，用于赛道一
    end_pose_suber = nh->subscribe<geometry_msgs::PoseStamped>("/airsim_node/end_pose", 1, std::bind(&PathSender::end_pose_cb, this, std::placeholders::_1));//状态真值，用于赛道一
     timer = nh->createTimer(ros::Duration(1.0),&PathSender::timeCB,this);

    //waypoint_publisher = nh->advertise<WayPoints::WayPoints>("/waypoints", 1);

    ros::spin();
}

PathSender::~PathSender()
{
}


void PathSender::initial_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{ 
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


void PathSender::timeCB(const ros::TimerEvent& event)
{
    if(!path_get)
    if(initial_num_get && end_num_get)
    {
        path.insert(path.end(), paths[initial_num-1].begin(), paths[initial_num-1].end());//将起点到第一个转运站的路径加入

        path.emplace_back(Transit_hub[initial_num]);//中枢入口
        path.emplace_back(Transit_hub[end_num]);//中枢出口

        std::vector<geometry_msgs::Point> temp_path=paths[end_num-1];
        std::reverse(temp_path.begin(),temp_path.end());//将终点到第转运站的路径倒序
        path.insert(path.end(),temp_path.begin(), temp_path.end());//将转运站到终点的路径加入

        path.emplace_back(end_point[end_num]);//终点后面一点

        geometry_msgs::Point temp_point;
        temp_point.x=end_point[end_num].x;
        temp_point.y=end_point[end_num].y;
        temp_point.z=end_point[end_num].z-150;
        path.emplace_back();//终点后面一点向上150m

        path.emplace_back(temp_point);//起点后面一点向上150m

        path.emplace_back(end_point[initial_num]);//起点后面一点

        path.emplace_back(station[initial_num]);//起点后面一点

        path_get=true;
    }
    else
    {
     // waypoint_publisher.publish(path);
    }

}




#endif