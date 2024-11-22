#include "param.hpp"

namespace ctrl_node{

    template <typename TName, typename TVal>
	void Parameter_t::read_param(const ros::NodeHandle &nh, const TName &name, TVal &val){
		if (!nh.getParam(name, val)){
			ROS_ERROR_STREAM("Read param: " << name << " failed.");
			ROS_BREAK();
		}
	};

    void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh){
        read_param(nh, "gain/Kvp0", gain.Kvp0);
        read_param(nh, "gain/Kvp1", gain.Kvp1);
        read_param(nh, "gain/Kvp2", gain.Kvp2);
        read_param(nh, "gain/Kvi0", gain.Kvi0);
        read_param(nh, "gain/Kvi1", gain.Kvi1);
        read_param(nh, "gain/Kvi2", gain.Kvi2);
        read_param(nh, "gain/Kvd0", gain.Kvd0);
        read_param(nh, "gain/Kvd1", gain.Kvd1);
        read_param(nh, "gain/Kvd2", gain.Kvd2);

        read_param(nh, "takeoff_state/height", takeoff_state.height);

        read_param(nh, "fsmparam/frequncy", fsmparam.frequncy);

        read_param(nh, "msg_timeout/odom", msg_timeout.odom);
        read_param(nh, "msg_timeout/cmd", msg_timeout.cmd);

        read_param(nh, "controller_type", controller_type);

        read_param(nh, "kine_cons/vel_ver_max", kine_cons.vel_ver_max);
        read_param(nh, "kine_cons/vel_hor_max", kine_cons.vel_hor_max);
        read_param(nh, "kine_cons/acc_ver_max", kine_cons.acc_ver_max);
        read_param(nh, "kine_cons/acc_hor_max", kine_cons.acc_hor_max);
    }
}