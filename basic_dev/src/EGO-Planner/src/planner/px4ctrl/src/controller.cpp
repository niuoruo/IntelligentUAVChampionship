#include "controller.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <uav_utils/converters.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <boost/format.hpp>
#include "std_msgs/Float32.h"
#include "airsim_ros/VelCmd.h"


using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

Controller::Controller(Parameter_t& param_):
	param(param_)
{
	is_configured = false;
	int_e_v.setZero();
}

void Controller::config()
{
	config_gain(param.hover_gain);
	is_configured = true;
}

void Controller::config_gain(const Parameter_t::Gain& gain)
{
	Kp.setZero();
	Kv.setZero();
	Ka.setZero();
	Kvi.setZero();
	Kp(0,0) = gain.Kp0;
	Kp(1,1) = gain.Kp1;
	Kp(2,2) = gain.Kp2;
	Kpd(0,0) = gain.Kpd0;
	Kpd(1,1) = gain.Kpd1;
	Kpd(2,2) = gain.Kpd2;
	Kv(0,0) = gain.Kv0;
	Kv(1,1) = gain.Kv1;
	Kv(2,2) = gain.Kv2;
	Kvi(0,0) = gain.Kvi0;
	Kvi(1,1) = gain.Kvi1;
	Kvi(2,2) = gain.Kvi2;
	Kvd(0,0) = gain.Kvd0;
	Kvd(1,1) = gain.Kvd1;
	Kvd(2,2) = gain.Kvd2;
	Ka(0,0) = gain.Ka0;
	Ka(1,1) = gain.Ka1;
	Ka(2,2) = gain.Ka2;
	Kyaw = gain.Kyaw;
}
Eigen::Quaterniond Controller::computeDesiredAttitude(
    const Eigen::Vector3d& desired_acceleration, const double reference_heading,
    const Eigen::Quaterniond& attitude_estimate) const{
	//desired_acceleration means the desired thrust and is perpendicular to the body frame.
	
	const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
    Eigen::AngleAxisd(reference_heading, Eigen::Vector3d::UnitZ()));
	// Compute desired orientation
  	const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  	const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
  	Eigen::Vector3d z_B;
  	if (almostZero(desired_acceleration.norm())) {
		// In case of free fall we keep the thrust direction to be the estimated one
		// This only works assuming that we are in this condition for a very short
		// time (otherwise attitude drifts)
    z_B = attitude_estimate * Eigen::Vector3d::UnitZ();
  	}	
	else {
    	z_B = desired_acceleration.normalized();
  	}

  	const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  	const Eigen::Vector3d x_B =
    computeRobustBodyXAxis(x_B_prototype, x_C, y_C, attitude_estimate);
  	const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();
  	// From the computed desired body axes we can now compose a desired attitude
	const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
	// ROS_INFO_STREAM("R_W_B: "<<R_W_B);
	const Eigen::Quaterniond desired_attitude(R_W_B);

	return desired_attitude;
}

Controller_Output_t Controller::computeNominalReferenceInputs(
    const Desired_State_t& reference_state,
    const Odom_Data_t& attitude_estimate) const{

	Controller_Output_t reference_command;
  	const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
    Eigen::AngleAxisd(reference_state.yaw, Eigen::Vector3d::UnitZ()));
  	const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
 		const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
  	const Eigen::Vector3d des_acc = reference_state.a + Vector3d(0, 0, param.gra);
	//desired thrust direction, the same as se3 planner

	//	F_des = u_v * param.mass + 
	//	Vector3d(0, 0, param.mass * param.gra) + Ka * param.mass * des.a;
  // Reference attitude
  	const Eigen::Quaterniond q_W_B = computeDesiredAttitude(
      des_acc, reference_state.yaw, attitude_estimate.q);
	//same as planner

  const Eigen::Vector3d x_B = q_W_B * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d y_B = q_W_B * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d z_B = q_W_B * Eigen::Vector3d::UnitZ();

  reference_command.orientation = q_W_B;

  // Reference thrust
  reference_command.normalized_thrust = des_acc.norm();

  // Reference body rates
  if (almostZeroThrust(reference_command.normalized_thrust)) {
    reference_command.roll_rate = 0.0;
    reference_command.pitch_rate = 0.0;
  } else {
    reference_command.roll_rate = 1.0 /
                                      reference_command.normalized_thrust *
                                      y_B.dot(reference_state.jerk);
    reference_command.pitch_rate = 1.0 /
                                      reference_command.normalized_thrust *
                                      x_B.dot(reference_state.jerk);
  }
//   reference_command.yaw_rate = 0.0;

  if (almostZero((y_C.cross(z_B)).norm())) {
    reference_command.yaw_rate = 0.0;
  } else {
    reference_command.yaw_rate =
        1.0 / (y_C.cross(z_B)).norm() *
        (reference_state.head_rate * x_C.dot(x_B) +
         reference_command.pitch_rate * y_C.dot(z_B));
  }

  return reference_command;

	}
	
void Controller::update(
	const Desired_State_t& des, 
	const Odom_Data_t& odom, 
	const Imu_Data_t& imu,
	Controller_Output_t& u, 
	SO3_Controller_Output_t& u_so3
)
{

	//hzchzc
	/*
	*step1
	Compute reference inputs
	*/
	 // Compute reference inputs
	std::string constraint_info("");
  	Eigen::Vector3d drag_accelerations = Eigen::Vector3d::Zero();
  	Controller_Output_t reference_inputs;
  	if (param.perform_aerodynamics_compensation) {
    // Compute reference inputs that compensate for aerodynamic drag
    // computeAeroCompensatedReferenceInputs(reference_state, state_estimate,
    //                                       config, &reference_inputs,
	// 									 	&drag_accelerations);
  	} 
	else {
		// In this case we are not considering aerodynamic accelerations
		drag_accelerations = Eigen::Vector3d::Zero();
		// Compute reference inputs as feed forward terms
		reference_inputs = computeNominalReferenceInputs(des, odom);
  	}

	/*
	step2 get the pid error
	*/
	// Vector3d F_des;
	// F_des = computePIDErrorAcc(odom,des)*param.mass;//backfeed
	// F_des += Vector3d(0, 0, param.mass * param.gra) + Ka * param.mass * des.a;//feed back
	// F_des -= drag_accelerations*param.mass;

	Vector3d e_p, e_v, F_des;
	double e_yaw = 0.0;
	if (des.v(0) != 0.0 || des.v(1) != 0.0 || des.v(2) != 0.0) {
		// ROS_INFO("Reset integration");
		int_e_v.setZero();
	}
	double yaw_curr = get_yaw_from_quaternion(odom.q);
	double	yaw_des = des.yaw;
	Matrix3d wRc = rotz(yaw_curr);
	Matrix3d cRw = wRc.transpose();
	e_p = des.p - odom.p;
	Eigen::Vector3d u_p;
	if (e_p.norm() > param.thresh)
		u_p = wRc * (param.Kp_gain * Kp) * cRw * e_p;
	else
		u_p = wRc * Kp * cRw * e_p;

  Eigen::Vector3d N0(0, 0, 1);
  Eigen::Vector3d N = odom.q * N0;
  double water_angle = std::atan2(std::sqrt(N.x() * N.x() + N.y() * N.y()), N.z());
	u_p(2) /= std::cos(water_angle);

	u.des_v_real = des.v + u_p; // For estimating hover percent
	e_v = des.v + u_p - odom.v;

	const std::vector<double> integration_enable_limits = {0.1, 0.1, 0.1};
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(e_v(k)) < 0.2) {
			int_e_v(k) += e_v(k) * 1.0 / 50.0;
		}
	}
	Eigen::Vector3d u_v_p = wRc * Kv * cRw * e_v;
	const std::vector<double> integration_output_limits = {0.6, 0.6, 0.6};
	const std::vector<double> d_output_limits = {1.6, 1.6, 1.6};
	Eigen::Vector3d u_v_i = wRc * Kvi * cRw * int_e_v;
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(u_v_i(k)) > integration_output_limits[k]) {
			uav_utils::limit_range(u_v_i(k), integration_output_limits[k]);
			ROS_INFO("Integration saturate for axis %zu, value=%.3f", k, u_v_i(k));
		}
	}
	Eigen::Matrix3d rot = odom.q.toRotationMatrix();
	Eigen::Vector3d a = imu.a - rot.inverse() * Eigen::Vector3d(0, 0, param.gra);
	// std::cout << "a:" << a.transpose() << std::endl;
	Eigen::Vector3d u_v_d = -wRc * Kvd * cRw * a;
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(u_v_d(k)) > d_output_limits[k]) {
			uav_utils::limit_range(u_v_d(k), d_output_limits[k]);
			ROS_INFO("d for axis %zu, value=%.3f", k, u_v_d(k));
		}
	}
	// std::cout << "u_v_d: " << u_v_d.transpose() << std::endl;
	Eigen::Vector3d u_v = u_v_p + u_v_i + u_v_d;
	// std::cout << "u_v: " << u_v.transpose() << std::endl;
	e_yaw = yaw_des - yaw_curr;
	while(e_yaw > M_PI) e_yaw -= (2 * M_PI);
	while(e_yaw < -M_PI) e_yaw += (2 * M_PI);
	double u_yaw = Kyaw * e_yaw;
	F_des = u_v * param.mass + 
		Vector3d(0, 0, param.mass * param.gra) + Ka * param.mass * des.a;
	// std::cout << "des.a: " << (Vector3d(0, 0, param.mass * param.gra)).transpose() << std::endl;
	// std::cout << "des.a: " << (Ka * param.mass * des.a).transpose() << std::endl;
	
	// std::cout << "F_des: " << F_des.transpose() << std::endl;

	Matrix3d wRb_odom = odom.q.toRotationMatrix();
	Vector3d z_b_curr = wRb_odom.col(2);
	// std::cout << "roll pitch yaw: " << odom.q.toRotationMatrix().eulerAngles(0, 1, 2).transpose() << std::endl;
	// std::cout << "z_b_curr: " << z_b_curr.transpose() << std::endl;
	u.thrust = F_des.dot(z_b_curr);

	const Eigen::Quaterniond desired_attitude = computeDesiredAttitude(F_des/param.mass, des.yaw, odom.q);
	const Eigen::Vector3d rpy = desired_attitude.toRotationMatrix().eulerAngles(0, 1, 2);
	// std::cout << "Desired Attitude - Roll: " << rpy.x() << ", Pitch: " << rpy.y() << ", Yaw: " << rpy.z() << std::endl;

	const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude, odom.q);
	// std::cout << "Feedback Bodyrates: " << feedback_bodyrates.transpose() << std::endl;
	u.roll_rate = reference_inputs.roll_rate+feedback_bodyrates.x();
	u.pitch_rate = reference_inputs.pitch_rate+feedback_bodyrates.y();
	u.yaw_rate = reference_inputs.yaw_rate+feedback_bodyrates.z();

	// ROS_INFO_STREAM("roll_rate: "<<u.roll_rate);
	double limit_rate = 6.0*3.14;
	if(u.roll_rate>=limit_rate)
		ROS_INFO("ROLL RATE LIMIT!");
	if(u.pitch_rate>=limit_rate)
		ROS_INFO("pitch_rate_limit!");

	uav_utils::limit_range(u.roll_rate,limit_rate);//3.0
	uav_utils::limit_range(u.pitch_rate,limit_rate);
	uav_utils::limit_range(u.yaw_rate,1.5);

	// std::cout << "u.roll_rate: " << u.roll_rate << std::endl;
	// std::cout << "u.pitch_rate: " << u.pitch_rate << std::endl;
	// std::cout << "u.yaw_rate: " << u.yaw_rate << std::endl;

	u.roll_rate = param.Krpd * param.Ixx * (u.roll_rate - odom.w(0));
	u.pitch_rate = param.Krpd * param.Iyy * (u.pitch_rate - odom.w(1));
	u.yaw_rate = param.Kyawd * param.Izz * (u.yaw_rate - odom.w(2));


	if(u.roll_rate>=limit_rate)
		ROS_INFO("2ROLL RATE LIMIT!");
	if(u.pitch_rate>=limit_rate)
		ROS_INFO("2pitch_rate_limit!");
	uav_utils::limit_range(u.roll_rate, 9.0*3.14);
	uav_utils::limit_range(u.pitch_rate, 9.0*3.14);
	uav_utils::limit_range(u.yaw_rate, 3.0);

	// printf("thrust: %f \n",u.thrust);
	// printf("roll: %f \n",u.roll_rate);
	// printf("pitch: %f \n",u.pitch_rate);
	// printf("yaw: %f \n",u.yaw_rate);
};

airsim_ros::RotorPWM Controller::computePWM(
	const double& thrust,
	const Eigen::Vector3d& torque)
{
	Eigen::Vector4d thrust_torque;
	thrust_torque << thrust, torque(0), torque(1), torque(2);

	Eigen::Vector4d rps_pow2_temp = param.allocation_matrix_inv * thrust_torque;

	double sqartpwm2rps = param.sqrtpwm2rads / (2 * M_PI);
	Eigen::Vector4d pwm_cal = rps_pow2_temp / (sqartpwm2rps * sqartpwm2rps);

	airsim_ros::RotorPWM pwm;
	pwm.header.stamp = ros::Time::now();
	pwm.rotorPWM0 = pwm_cal(0);
	pwm.rotorPWM1 = pwm_cal(1);
	pwm.rotorPWM2 = pwm_cal(2);
	pwm.rotorPWM3 = pwm_cal(3);

	return pwm;
}

void Controller::publish_ctrl(const Controller_Output_t& u, const ros::Time& stamp, bool is_init,Command_Data_t& cmd_data,float x,float y,float z,float w)
{

	static int cnt = 0;
    Eigen::Quaterniond q(w,x,y,z);
	Eigen::Matrix3d rotationMatrix=q.toRotationMatrix();
    Eigen::Vector3d originalPoint(cmd_data.v(0),cmd_data.v(1),cmd_data.v(2));
	Eigen::Matrix3d tran;
	tran <<
      1.0, 0.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0
		;

    rotationMatrix = tran * rotationMatrix * tran;
    Eigen::Vector3d rotatedPoint = rotationMatrix.inverse() * originalPoint;
if (cnt < 100)
	{
		std::cout << "here" << std::endl;
		airsim_ros::RotorPWM pwm;
		pwm.rotorPWM0 = 0.178087130188 + 0.03;
		pwm.rotorPWM1 = 0.178087130188 + 0.03;
		pwm.rotorPWM2 = 0.178087130188 + 0.03;
		pwm.rotorPWM3 = 0.178087130188 + 0.03;
		ctrl_PWM_pub.publish(pwm);
		cnt++;
		return;
	}
	else if (cnt >= 100&& cnt < 180)
	{
		std::cout << "here" << std::endl;
		airsim_ros::RotorPWM pwm;
		pwm.rotorPWM0 = 0.178087130188 - 0.03;
		pwm.rotorPWM1 = 0.178087130188 - 0.03;
		pwm.rotorPWM2 = 0.178087130188 - 0.03;
		pwm.rotorPWM3 = 0.178087130188 - 0.03;
		ctrl_PWM_pub.publish(pwm);
		cnt++;
		return;
	}
	else if (!is_init)
	{
		airsim_ros::RotorPWM pwm;
		pwm.rotorPWM0 = 0.178087130188;
		pwm.rotorPWM1 = 0.178087130188;
		pwm.rotorPWM2 = 0.178087130188;
		pwm.rotorPWM3 = 0.178087130188;
		ctrl_PWM_pub.publish(pwm);
		return;
	}	
	float yaw = q.toRotationMatrix().eulerAngles(2,1,0)[0];
	float pitch = q.toRotationMatrix().eulerAngles(2,1,0)[1];
	// msg.header.stamp = stamp;
	// msg.header.frame_id = std::string("FCU");
	// msg.body_rate.x = u.roll_rate;
	// msg.body_rate.y = u.pitch_rate;
	// msg.body_rate.z = u.yaw_rate;
	// msg.thrust = u.thrust;
    //std::cout<<"yaw"<<yaw<<std::endl;
	//std::cout<<"pitch"<<pitch<<std::endl;
  // ctrl_FCU_pub.publish(msg);

	Eigen::Vector3d torque;
	torque << u.roll_rate, u.pitch_rate, u.yaw_rate;

	airsim_ros::RotorPWM pwm;
	airsim_ros::VelCmd vel_cmd;
	pwm = computePWM(u.thrust, torque);
	
    vel_cmd.twist.linear.x =  (-cmd_data.v(0)*cos(yaw)+cmd_data.v(1)*sin(yaw))*(-cos(pitch))-cmd_data.v(2)*sin(pitch);
	vel_cmd.twist.linear.y = (cmd_data.v(1)*cos(yaw)+cmd_data.v(0)*sin(yaw));
	// std::cout<<"vel_cmd.twist.linear.x"<<vel_cmd.twist.linear.x<<std::endl;
	// std::cout<<"vel_cmd.twist.linear.y"<<vel_cmd.twist.linear.y<<std::endl;
	vel_cmd.twist.linear.z = cmd_data.v(2)*cos(pitch);//(-cmd_data.v(0)*cos(yaw)+cmd_data.v(1)*sin(yaw))*cos(pitch);
    vel_cmd.twist.linear.x = rotatedPoint(0)*1.5;
	vel_cmd.twist.linear.y = -rotatedPoint(1)*1.5;
	vel_cmd.twist.linear.z = -rotatedPoint(2)*1.5;	
	// if (cnt < 100)
	// {
	// 	vel_cmd.twist.linear.z = -0.8;


	//     vel_pub.publish(vel_cmd);
	// 	cnt++;
	// }
	ctrl_PWM_pub.publish(pwm);
		//std::cout<<"vel_cmd.twist.linear.z"<<vel_cmd.twist.linear.z<<std::endl;

	//vel_pub.publish(vel_cmd);

}
void Controller::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    x=msg->pose.orientation.x;
	y=msg->pose.orientation.y;
	z=msg->pose.orientation.z;
	w=msg->pose.orientation.w;
	// Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2,1,0);
	// ROS_INFO("Get pose data. time: %f, eulerangle: %f, %f, %f, posi: %f, %f, %f\n", msg->header.stamp.sec + msg->header.stamp.nsec*1e-9,
	// 	eulerAngle[0], eulerAngle[1], eulerAngle[2], msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void Controller::publish_zero_ctrl(const ros::Time& stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE  | 
					mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE | 
					mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	Eigen::Vector3d uAngle(0.0, 0.0, 0.0);
	Eigen::Quaterniond quaternion = ypr_to_quaternion(uAngle);
	// Eigen::Quaterniond quaternion;
	// quaternion = Eigen::AngleAxisd(AngleAxisd(uAngle(0),Vector3d::UnitZ())) * 
	// 			 Eigen::AngleAxisd(AngleAxisd(uAngle(1),Vector3d::UnitY())) * 
	// 			 Eigen::AngleAxisd(AngleAxisd(uAngle(2),Vector3d::UnitX()));
	msg.orientation.x = quaternion.x();
	msg.orientation.y = quaternion.y();
	msg.orientation.z = quaternion.z();
	msg.orientation.w = quaternion.w();
	
	msg.thrust = param.hov_percent*0.8;
	cout<<"I published zero control!!!!!!!"<<endl;
	
    ctrl_FCU_pub.publish(msg);
}
bool Controller::almostZero(const double value) const{
	return fabs(value) < 0.001; 
}
bool Controller::almostZeroThrust(const double thrust_value) const {
  return fabs(thrust_value) < 0.01;
}
Eigen::Vector3d Controller::computeRobustBodyXAxis(
    const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Quaterniond& attitude_estimate) const {
  Eigen::Vector3d x_B = x_B_prototype;

  if (almostZero(x_B.norm())) {
		std::cout << "here" << std::endl;
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated =
        attitude_estimate * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (almostZero(x_B_projected.norm())) {
      // Not too much intelligent stuff we can do in this case but it should
      // basically never occur
      x_B = x_C;
    } else {
      x_B = x_B_projected.normalized();
    }
  } else {
    x_B.normalize();
  }

  // if the quad is upside down, x_B will point in the "opposite" direction
  // of x_C => flip x_B (unfortunately also not the solution for our problems)
  //  if (x_B.dot(x_C) < 0.0)
  //  {
  //    x_B = -x_B;
  //  }

  return x_B;
}
Eigen::Vector3d Controller::computeFeedBackControlBodyrates(const Eigen::Quaterniond& desired_attitude,
    const Eigen::Quaterniond& attitude_estimate){
		  // Compute the error quaternion
  const Eigen::Quaterniond q_e = attitude_estimate.inverse() * desired_attitude;
  // Compute desired body rates from control error
  Eigen::Vector3d bodyrates;
  double krp = param.track_gain.Krp;
  double kyaw = param.track_gain.Kyaw;
  double kdrp = param.Kdrp;
  double kdyaw = param.Kdyaw;

	static Eigen::Vector3d last_rate = Eigen::Vector3d::Zero();

  if (q_e.w() >= 0) {
    bodyrates.x() = 2.0 * krp * q_e.x();
    bodyrates.y() = 2.0 * krp * q_e.y();
    bodyrates.z() = 2.0 * kyaw * q_e.z();
  } else {
    bodyrates.x() = -2.0 * krp * q_e.x();
    bodyrates.y() = -2.0 * krp * q_e.y();
    bodyrates.z() = -2.0 * kyaw * q_e.z();
  }

	bodyrates.x() += kdrp * (bodyrates.x() - last_rate.x());
	bodyrates.y() += kdrp * (bodyrates.y() - last_rate.y());
	bodyrates.z() += kdyaw * (bodyrates.z() - last_rate.z());
		
	last_rate = bodyrates;

  return bodyrates;
}
Eigen::Vector3d Controller::computePIDErrorAcc(
    const Odom_Data_t& state_estimate,
    const Desired_State_t& reference_state){
	  // Compute the desired accelerations due to control errors in world frame
  // with a PID controller
  Eigen::Vector3d acc_error;

  // x acceleration
  double x_pos_error =
      reference_state.p[0] - state_estimate.p[0];
  //uav_utils::limit_range(u_v_i(k), integration_output_limits[k]);
  uav_utils::limit_range(x_pos_error,param.pxy_error_max);

  double x_vel_error =
      reference_state.v[0] - state_estimate.v[0];
  uav_utils::limit_range(x_vel_error,param.vxy_error_max);

  acc_error.x() = Kp(0,0) * x_pos_error + Kv(0,0) * x_vel_error;
  // y acceleration
  double y_pos_error =
      reference_state.p[1] - state_estimate.p[1];
  uav_utils::limit_range(y_pos_error,param.pxy_error_max);

  double y_vel_error =
      reference_state.v[1]- state_estimate.v[1];
  uav_utils::limit_range(y_vel_error,param.vxy_error_max);
  acc_error.y() = Kp(1,1) * y_pos_error + Kv(1,1) * y_vel_error;
  // z acceleration
  double z_pos_error =
      reference_state.p[2] - state_estimate.p[2];
  uav_utils::limit_range(z_pos_error,
                          param.pz_error_max);

  double z_vel_error =
      reference_state.v[2] - state_estimate.v[2];
  uav_utils::limit_range(z_vel_error, param.vz_error_max);

  acc_error.z() = Kp(2,2) * z_pos_error + Kv(2,2) * z_vel_error;

  return acc_error;
}