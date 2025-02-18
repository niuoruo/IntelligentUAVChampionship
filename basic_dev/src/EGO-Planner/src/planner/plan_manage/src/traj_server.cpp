#include <nav_msgs/Odometry.h>
#include <traj_utils/PolyTraj.h>
#include <optimizer/poly_traj_utils.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include "airsim_ros/VelCmd.h"
#include "airsim_ros/PoseCmd.h"

using namespace Eigen;

ros::Publisher pos_cmd_pub;

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
double yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;

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
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1
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

  const double YDM = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double YDDM = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * YDDM) <= fabs(YDM))
  {
    // yawdot = last_yawdot_ + dt * YDDM;
    d_yaw_max = last_yawdot_ * dt + 0.5 * YDDM * dt * dt;
  }
  else
  {
    // yawdot = YDM;
    double t1 = (YDM - last_yawdot_) / YDDM;
    d_yaw_max = ((dt - t1) + dt) * (YDM - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
  {
    d_yaw = d_yaw_max;
  }
  yawdot = d_yaw / dt;

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
  cmd.twist.linear.x = v(0);  // x方向线速度(m/s)
  cmd.twist.linear.y = -v(1);  // y方向线速度(m/s)
  cmd.twist.linear.z = -v(2);  // z方向线速度(m/s)
  cmd.twist.angular.z = -yd;   // z方向角速度(yaw, deg)

  // cmd.position.x = p(0);
  // cmd.position.y = p(1);
  // cmd.position.z = p(2);
  // cmd.velocity.x = v(0);
  // cmd.velocity.y = v(1);
  // cmd.velocity.z = v(2);
  // cmd.acceleration.x = a(0);
  // cmd.acceleration.y = a(1);
  // cmd.acceleration.z = a(2);
  // cmd.jerk.x = j(0);
  // cmd.jerk.y = j(1);
  // cmd.jerk.z = j(2);
  // cmd.yaw = y;
  // cmd.yaw_dot = yd;
  pos_cmd_pub.publish(cmd);

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
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  yaw_ = yaw;
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
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), yaw_, 0);
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


  // Calculate yaw
  yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());
  yaw_yawdot.second = 0.0;

  double cos_yaw = cos(yaw_);
  double sin_yaw = sin(yaw_);
  Eigen::Vector3d vel_drone;
  vel_drone(0) = cos_yaw * vel(0) + sin_yaw * vel(1);
  vel_drone(1) = -sin_yaw * vel(0) + cos_yaw * vel(1);
  vel_drone(2) = vel(2);

  vel = vel_drone;

  /*** calculate yaw ***/
  yaw_yawdot = calculate_yaw(t_cur, pos, (time_now - time_last).toSec());
  // Update last values
  time_last = time_now;
  last_pos_ = pos;

    // publish
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
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

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  nh.param("traj_server/time_forward", time_forward_, -1.0);
  yaw_ = 0.0;
  last_yawdot_ = 0.0;

  ros::Duration(1.0).sleep();

  ROS_INFO("[Traj server]: ready.");

  ros::spin();

  return 0;
}