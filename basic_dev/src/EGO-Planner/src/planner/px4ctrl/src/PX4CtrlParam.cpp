#include "PX4CtrlParam.h"

Parameter_t::Parameter_t()
{

}

void Parameter_t::config_from_ros_handle(const ros::NodeHandle& nh)
{
	read_essential_param(nh, "gain/track/Kp0", hover_gain.Kp0);
	read_essential_param(nh, "gain/track/Kp1", hover_gain.Kp1);
	read_essential_param(nh, "gain/track/Kp2", hover_gain.Kp2);
	
	read_essential_param(nh, "gain/track/Kv0", hover_gain.Kv0);
	read_essential_param(nh, "gain/track/Kv1", hover_gain.Kv1);
	read_essential_param(nh, "gain/track/Kv2", hover_gain.Kv2);

	read_essential_param(nh, "gain/track/Kvi0", hover_gain.Kvi0);
	read_essential_param(nh, "gain/track/Kvi1", hover_gain.Kvi1);
	read_essential_param(nh, "gain/track/Kvi2", hover_gain.Kvi2);

	read_essential_param(nh, "gain/track/Ka0", hover_gain.Ka0);
	read_essential_param(nh, "gain/track/Ka1", hover_gain.Ka1);
	read_essential_param(nh, "gain/track/Ka2", hover_gain.Ka2);

	read_essential_param(nh, "gain/track/Kyaw", hover_gain.Kyaw);
	read_essential_param(nh, "gain/track/Krp", hover_gain.Krp);
	

	read_essential_param(nh, "gain/track/Kp0", track_gain.Kp0);
	read_essential_param(nh, "gain/track/Kp1", track_gain.Kp1);
	read_essential_param(nh, "gain/track/Kp2", track_gain.Kp2);
	
	read_essential_param(nh, "gain/track/Kpd0", track_gain.Kpd0);
	read_essential_param(nh, "gain/track/Kpd1", track_gain.Kpd1);
	read_essential_param(nh, "gain/track/Kpd2", track_gain.Kpd2);

	read_essential_param(nh, "gain/track/Kv0", track_gain.Kv0);
	read_essential_param(nh, "gain/track/Kv1", track_gain.Kv1);
	read_essential_param(nh, "gain/track/Kv2", track_gain.Kv2);

	read_essential_param(nh, "gain/track/Kvi0", track_gain.Kvi0);
	read_essential_param(nh, "gain/track/Kvi1", track_gain.Kvi1);
	read_essential_param(nh, "gain/track/Kvi2", track_gain.Kvi2);

	read_essential_param(nh, "gain/track/Kvd0", track_gain.Kvd0);
	read_essential_param(nh, "gain/track/Kvd1", track_gain.Kvd1);
	read_essential_param(nh, "gain/track/Kvd2", track_gain.Kvd2);

	read_essential_param(nh, "gain/track/Ka0", track_gain.Ka0);
	read_essential_param(nh, "gain/track/Ka1", track_gain.Ka1);
	read_essential_param(nh, "gain/track/Ka2", track_gain.Ka2);

	read_essential_param(nh, "gain/track/Kyaw", track_gain.Kyaw);
	read_essential_param(nh, "gain/track/Krp", track_gain.Krp);

	read_essential_param(nh, "gain/track/Kp_gain", Kp_gain);
	read_essential_param(nh, "gain/track/thresh", thresh);
	read_essential_param(nh, "gain/track/Kdrp", Kdrp);
	read_essential_param(nh, "gain/track/Kdyaw", Kdyaw);
	read_essential_param(nh, "gain/track/Kyawd", Kyawd);
	read_essential_param(nh, "gain/track/Krpd", Krpd);

	read_essential_param(nh, "hover/use_hov_percent_kf", hover.use_hov_percent_kf);
	read_essential_param(nh, "hover/percent_lower_limit", hover.percent_lower_limit);
	read_essential_param(nh, "hover/percent_higher_limit", hover.percent_higher_limit);

	read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
	read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
	read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
	read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);

	read_essential_param(nh, "mass", mass);
	read_essential_param(nh, "gra", gra);
	read_essential_param(nh, "hov_percent", hov_percent);
	read_essential_param(nh, "full_thrust", full_thrust);
	read_essential_param(nh, "ctrl_rate", ctrl_rate);
	read_essential_param(nh, "use_yaw_rate_ctrl", use_yaw_rate_ctrl);
	read_essential_param(nh,"perform_aerodynamics_compensation",perform_aerodynamics_compensation);
	read_essential_param(nh,"pxy_error_max",pxy_error_max);
	read_essential_param(nh,"vxy_error_max",vxy_error_max);
	read_essential_param(nh,"pz_error_max",pz_error_max);
	read_essential_param(nh,"vz_error_max",vz_error_max);
	read_essential_param(nh,"yaw_error_max",yaw_error_max);

	read_essential_param(nh, "pwm_param/C_t", C_t);
	read_essential_param(nh, "pwm_param/C_M", C_M);
	read_essential_param(nh, "pwm_param/arm_length", arm_length);
	read_essential_param(nh, "pwm_param/sqrtpwm2rads", sqrtpwm2rads);
	read_essential_param(nh, "pwm_param/Ixx", Ixx);
	read_essential_param(nh, "pwm_param/Iyy", Iyy);
	read_essential_param(nh, "pwm_param/Izz", Izz);
};

void Parameter_t::init()
{
    // 0:右前 1:左后 2:左前 3:右后
    double temp_Ct = C_t;
    double temp_Ct_arm_sqrt2 = 0.7071 * arm_length * C_t;
    double temp_CM = C_M;
    allocation_matrix << temp_Ct, temp_Ct, temp_Ct, temp_Ct,
												-temp_Ct_arm_sqrt2, temp_Ct_arm_sqrt2, temp_Ct_arm_sqrt2, -temp_Ct_arm_sqrt2,
												-temp_Ct_arm_sqrt2, temp_Ct_arm_sqrt2, -temp_Ct_arm_sqrt2, temp_Ct_arm_sqrt2,
												-temp_CM, -temp_CM, temp_CM, temp_CM;
    allocation_matrix_inv = allocation_matrix.inverse();
};

void Parameter_t::config_full_thrust(double hov)
{
	full_thrust = hover.use_hov_percent_kf ? (mass * gra / hov) : full_thrust;
};
