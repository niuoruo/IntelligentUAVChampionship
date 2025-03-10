#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <airsim_ros/RotorPWM.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"
#include <Eigen/Dense>

using namespace Eigen;

ros::Publisher pos_cmd_pub;
ros::Publisher pwm_pub;
ros::Publisher position_cmd_pub;

airsim_ros::VelCmd cmd;
// double pos_gain[3] = {0, 0, 0};
// double vel_gain[3] = {0, 0, 0};

#define FLIP_YAW_AT_END 0
#define TURN_YAW_TO_CENTER_AT_END 0

bool receive_traj_ = false;
boost::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
double traj_time_;
ros::Time start_time_;
int traj_id_;
ros::Time heartbeat_time_(0);
Eigen::Vector3d last_pos_;
nav_msgs::Odometry odom_;

// yaw control
double roll_, pitch_, yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double first_yaw_;
bool odom_inited = false;
double time_forward_;

Eigen::VectorXf X_des, X_real;

void heartbeatCallback(std_msgs::EmptyPtr msg)
{
  heartbeat_time_ = ros::Time::now();
}

void polyTrajCallback(traj_utils::PolyTrajPtr msg)
{
  if (msg->order != 5)
  {
    ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    ROS_ERROR("[traj_server] WRONG trajectory parameters, ");
    return;
  }

  int piece_nums = msg->duration.size();
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];

    dura[i] = msg->duration[i];
  }

  traj_.reset(new poly_traj::Trajectory(dura, cMats));

  start_time_ = msg->start_time;
  traj_duration_ = traj_->getTotalDuration();
  traj_time_ = 0;
  traj_id_ = msg->traj_id;

  receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  // constexpr double YAW_DOT_MAX_PER_SEC = 2.0 * M_PI;
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5.0 * M_PI;
  constexpr double YAW_DOT_MAX_PER_SEC = M_PI / 2.0;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = M_PI / 2.0;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;

  // Create and publish visualization marker
  static ros::Publisher marker_pub = ros::NodeHandle().advertise<visualization_msgs::Marker>("future_position", 10);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0; 
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  if (t_cur + time_forward_ <= traj_duration_)
  {
    marker.pose.position.x = traj_->getPos(t_cur + time_forward_)(0);
    marker.pose.position.y = traj_->getPos(t_cur + time_forward_)(1);
    marker.pose.position.z = traj_->getPos(t_cur + time_forward_)(2);
  }
  else
  {
    marker.pose.position.x = traj_->getPos(traj_duration_)(0);
    marker.pose.position.y = traj_->getPos(traj_duration_)(1);
    marker.pose.position.z = traj_->getPos(traj_duration_)(2);
  }

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  marker_pub.publish(marker);
  
  double yaw_temp = dir.norm() > 0.05
                        ? atan2(dir(1), dir(0))
                        : yaw_;

  double yawdot = 0;
  double d_yaw = yaw_temp - yaw_;
  if (d_yaw >= M_PI)
  {
    d_yaw -= 2 * M_PI;
  }
  if (d_yaw <= -M_PI)
  {
    d_yaw += 2 * M_PI;
  }

  yawdot = (d_yaw > 0 ? 1 : -1) * sqrt(sqrt(fabs(d_yaw)));

  double yaw = yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;
  yaw_yawdot.first = yaw_temp;
  yaw_yawdot.second = yawdot;

  last_yawdot_ = yaw_yawdot.second;

  return yaw_yawdot;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{
  // Publish velocity command
  cmd.twist.linear.x = v(0);  // x方向线速度(m/s)
  cmd.twist.linear.y = -v(1);  // y方向线速度(m/s)
  cmd.twist.linear.z = -v(2);  // z方向线速度(m/s)
  cmd.twist.angular.z = -yd;   // z方向角速度(yaw, deg)
  pos_cmd_pub.publish(cmd);

  // airsim_ros::RotorPWM msg;
  // velocityToPWM(v(0), -v(1), -v(2),
  //            -yd, msg);
  // pwm_pub.publish(msg);

  // Publish velocity vectors for visualization
  static ros::Publisher vel_vis_pub = ros::NodeHandle().advertise<visualization_msgs::Marker>("/velocity_vector", 10);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "body";
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  
  // Start point of arrow (current position)
  marker.points.push_back(geometry_msgs::Point());
  marker.points[0].x = 0.0;
  marker.points[0].y = 0.0;
  marker.points[0].z = 0.0;
  
  // End point of arrow (velocity vector)
  marker.points.push_back(geometry_msgs::Point());
  marker.points[1].x = v(0) * 10;
  marker.points[1].y = v(1) * 10;
  marker.points[1].z = v(2) * 10;
  
  // Arrow properties
  marker.scale.x = 0.1;  // shaft diameter
  marker.scale.y = 0.2;  // head diameter
  marker.scale.z = 0.2;  // head length
  
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  
  vel_vis_pub.publish(marker);

  last_pos_ = p;
}

void odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
  odom_ = *msg;

  tf::Quaternion q(
    odom_.pose.pose.orientation.x,
    odom_.pose.pose.orientation.y,
    odom_.pose.pose.orientation.z,
    odom_.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll_, pitch_, yaw_);

  if (!odom_inited)
    first_yaw_ = yaw_;

    odom_inited = true;

}

void cmdCallback(const ros::TimerEvent &e)
{
  /* no publishing before receive traj_ and have heartbeat */
  if (heartbeat_time_.toSec() <= 1e-5)
  {
    // ROS_ERROR_ONCE("[traj_server] No heartbeat from the planner received");
    return;
  }
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();

  if ((time_now - heartbeat_time_).toSec() > 0.5)
  {
    ROS_ERROR("[traj_server] Lost heartbeat from the planner, is it dead?");

    receive_traj_ = false;
    // publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), yaw_, 0);
  }

  double t_cur = (time_now - start_time_).toSec();

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
/*

  // Get current position from odometry
  Eigen::Vector3d current_pos(odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);

  // PID gains
  static double kp = 1.0, ki = 0.0, kd = 0.1;
  static Eigen::Vector3d integral_error = Eigen::Vector3d::Zero();
  static Eigen::Vector3d last_error = Eigen::Vector3d::Zero();

  // Find the closest point on the trajectory
  for (; traj_time_ <= traj_duration_; traj_time_ += 0.1)
  {
    pos = traj_->getPos(traj_time_);
    double dist = (pos - current_pos).norm();
    if (dist > 0.1)
      break;
  }

  // PID control
  Eigen::Vector3d error = pos - current_pos;
  integral_error += error * (time_now - time_last).toSec();
  Eigen::Vector3d derivative_error = (error - last_error) / (time_now - time_last).toSec();
  vel = kp * error + ki * integral_error + kd * derivative_error;
*/

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);
  }
  else
  {
    pos = traj_->getPos(traj_duration_);
    vel = Eigen::Vector3d::Zero();
    acc = Eigen::Vector3d::Zero();
    jer = Eigen::Vector3d::Zero();
  }

  quadrotor_msgs::PositionCommand pos_cmd;
  pos_cmd.header.stamp = time_now;
  pos_cmd.header.frame_id = "odom";
  pos_cmd.position.x = pos(0);
  pos_cmd.position.y = pos(1);
  pos_cmd.position.z = pos(2);
  pos_cmd.velocity.x = vel(0);
  pos_cmd.velocity.y = vel(1);
  pos_cmd.velocity.z = vel(2);
  pos_cmd.acceleration.x = acc(0);
  pos_cmd.acceleration.y = acc(1);
  pos_cmd.acceleration.z = acc(2);
  pos_cmd.jerk.x = jer(0);
  pos_cmd.jerk.y = jer(1);
  pos_cmd.jerk.z = jer(2);
  pos_cmd.yaw = first_yaw_;
  position_cmd_pub.publish(pos_cmd);

  // quadrotor_msgs::PositionCommand pos_cmd;
  // pos_cmd.header.stamp = time_now;
  // pos_cmd.header.frame_id = "odom";
  // pos_cmd.position.x = 0.0;
  // pos_cmd.position.y = 0.0;
  // pos_cmd.position.z = 0.0;
  // pos_cmd.velocity.x = 2.0;
  // pos_cmd.velocity.y = 0.0;
  // pos_cmd.velocity.z = 0.0;
  // pos_cmd.acceleration.x = 0.0;
  // pos_cmd.acceleration.y = 0.0;
  // pos_cmd.acceleration.z = 0.0;
  // pos_cmd.jerk.x = 0.0;
  // pos_cmd.jerk.y = 0.0;
  // pos_cmd.jerk.z = 0.0;
  // yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());
  // pos_cmd.yaw = 0.0;
  // position_cmd_pub.publish(pos_cmd);

  time_last = time_now;

  // double cos_yaw = cos(yaw_);
  // double sin_yaw = sin(yaw_);
  // Eigen::Vector3d vel_drone;
  // vel_drone(0) = cos_yaw * vel(0) + sin_yaw * vel(1);
  // vel_drone(1) = -sin_yaw * vel(0) + cos_yaw * vel(1);
  // vel_drone(2) = vel(2);

  // vel = vel_drone;

  // /*** calculate yaw ***/
  // yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());
  // // Update last values
  // time_last = time_now;
  // last_pos_ = pos;

  //   // publish
  //   publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  // ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("planning/trajectory", 10, polyTrajCallback);
  ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);
  ros::Subscriber odomtry_sub = nh.subscribe("/Odometry", 10, odometryCallback);


  pos_cmd_pub = nh.advertise<airsim_ros::VelCmd>("/airsim_node/drone_1/vel_cmd_body_frame", 50);
  pwm_pub = nh.advertise<airsim_ros::RotorPWM>("/airsim_node/drone_1/rotor_pwm_cmd", 10);
  position_cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_command", 10);
  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  yaw_ = 0.0;
  last_yawdot_ = 0.0;

  X_des.resize(12);
  X_real.resize(12);

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}