#ifndef __READPARAM_HPP
#define __READPARAM_HPP

#include <ros/ros.h>

namespace ctrl_node{

    class Parameter_t{
        private:

        template <typename TName, typename TVal>
        void read_param(const ros::NodeHandle &nh, const TName &name, TVal &val);

        public:
        struct Gain{
            double Kvp0, Kvp1, Kvp2;
            double Kvi0, Kvi1, Kvi2;
            double Kvd0, Kvd1, Kvd2;
        };

        struct AutoTakeoff{
            double height;
        };

        struct kinematicsConstains{
            double vel_ver_max;
            double vel_hor_max;
        };

        struct FsmParam{
            int frequncy;
        };

        struct MsgTimeOut{
            double odom;
            double cmd;
        };

        int controller_type = 0; // 0 for position control, 1 for velocity control
        Gain gain;
        kinematicsConstains kine_cons;
        AutoTakeoff takeoff_state;
        FsmParam fsmparam;
        MsgTimeOut msg_timeout;
        void config_from_ros_handle(const ros::NodeHandle &nh);
    };
}
#endif