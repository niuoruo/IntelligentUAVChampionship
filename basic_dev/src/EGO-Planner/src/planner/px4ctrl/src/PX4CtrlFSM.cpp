#include "PX4CtrlFSM.h"
#include <uav_utils/converters.h>

using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

PX4CtrlFSM::PX4CtrlFSM(Parameter_t& param_, Controller& controller_, HovThrKF& hov_thr_kf_):
	param(param_), controller(controller_), hov_thr_kf(hov_thr_kf_)
{
	state = MANUAL_CTRL;
	hover_pose.setZero();
}

void PX4CtrlFSM::process()
{
	ros::Time now_time = ros::Time::now();
	Controller_Output_t u;
	SO3_Controller_Output_t u_so3;

	controller.config_gain(param.track_gain);
	process_cmd_control(u, u_so3);

	controller.publish_ctrl(u, now_time);
	hov_thr_kf.simple_update(u.des_v_real, odom_data.v );
	// This line may not take effect according to param.hov.use_hov_percent_kf
	param.config_full_thrust(hov_thr_kf.get_hov_thr());
}

void PX4CtrlFSM::publish_trigger(const nav_msgs::Odometry& odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;
	
    traj_start_trigger_pub.publish(msg);
}

bool PX4CtrlFSM::rc_is_received(const ros::Time& now_time)
{
	return (now_time - rc_data.rcv_stamp).toSec() < param.msg_timeout.rc;
}

bool PX4CtrlFSM::cmd_is_received(const ros::Time& now_time)
{
	return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
}

bool PX4CtrlFSM::odom_is_received(const ros::Time& now_time)
{
	return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
}

bool PX4CtrlFSM::imu_is_received(const ros::Time& now_time)
{
	return (now_time - imu_data.rcv_stamp).toSec() < param.msg_timeout.imu;
}

double PX4CtrlFSM::get_yaw_from_odom()
{
	return get_yaw_from_quaternion(odom_data.q);
}

void PX4CtrlFSM::process_hover_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	des.p = hover_pose.head<3>();
	des.v = Vector3d::Zero();
	des.yaw = hover_pose(3);
	des.a = Vector3d::Zero();
	des.jerk = Vector3d::Zero();
	controller.update(des, odom_data, imu_data, u, u_so3);

	//publish_desire(des);
}

void PX4CtrlFSM::process_cmd_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
	Desired_State_t des;
	des.p = cmd_data.p;
	des.v = cmd_data.v;
	des.yaw = cmd_data.yaw;
	des.a = cmd_data.a;
	des.jerk = cmd_data.jerk;
	des.head_rate = cmd_data.head_rate;
	// ROS_INFO_STREAM("desp: "<<des.p<<" desv: "<<des.v<<" desyaw: "<<des.yaw<<" desa "<<des.a);
	// ROS_INFO_STREAM("odomp: "<<odom_data.p<<" odomv: "<<odom_data.v);
	// ROS_INFO_STREAM("qo "<<odom_data.q.w()<<" q1 "<<odom_data.q.x() <<" q2 "<<odom_data.q.y()<<" q3 "<<odom_data.q.z());
	controller.update(des, odom_data, imu_data, u, u_so3);
	// ROS_INFO_STREAM("pitch: "<<u.pitch<<"roll: "<<u.roll<<" u.yaw: "<<u.yaw);

	//publish_desire(des);	
}

void PX4CtrlFSM::align_with_imu(Controller_Output_t& u)
{
	double imu_yaw = get_yaw_from_quaternion(imu_data.q); 
	double odom_yaw = get_yaw_from_odom();
	double des_yaw = u.yaw;
	// ROS_INFO_STREAM("imu yaw: "<<imu_yaw<<" odom_yaw: "<<odom_yaw);
	u.yaw = yaw_add(yaw_add(des_yaw, -odom_yaw), imu_yaw); 

	//out << "imu_yaw=" << imu_yaw << " odom_yaw=" << odom_yaw << " des_yaw=" << des_yaw << " u.yaw=" << u.yaw << endl;
};

void PX4CtrlFSM::set_hov_with_odom()
{
	hover_pose.head<3>() = odom_data.p;
	hover_pose(3) = get_yaw_from_odom();
}

void PX4CtrlFSM::toggle_offboard_mode(bool on_off)
{	
	mavros_msgs::SetMode offb_set_mode;
	ros::Time last_request = ros::Time::now();

	if ( on_off )
	{
		offb_set_mode.request.custom_mode = "OFFBOARD";
		controller.set_FCU_mode.call(offb_set_mode);
	}
	else
	{
		offb_set_mode.request.custom_mode = "ALTCTL";
		controller.set_FCU_mode.call(offb_set_mode);
	}
	
}
bool PX4CtrlFSM::px4_init(){
	return (odom_data.odom_init&&imu_data.imu_init&&cmd_data.cmd_init);
}
