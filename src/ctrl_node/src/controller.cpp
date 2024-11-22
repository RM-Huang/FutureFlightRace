#include "controller.hpp"

namespace Controller{
    void Velocity_Control::init(ctrl_node::Parameter_t &param){
        param_ = param;
    }

    double Velocity_Control::get_vel_err(const Desired_State_t &des, const ctrl_node::Odom_Data_t &odom){
        Eigen::Vector3d des_v = Kp.asDiagonal() * (des.p - odom.p);

    }

    quadrotor_msgs::Px4ctrlDebug Velocity_Control::calculateControl(const Desired_State_t &des, 
                                                                    const ctrl_node::Odom_Data_t &odom, 
                                                                    VP_Controller_Output_t &u){
    // 
        Eigen::Vector3d Kp;
        Kp << param_.gain.Kvp0, param_.gain.Kvp1, param_.gain.Kvp2;

        Eigen::Vector3d acc_max(param_.kine_cons.acc_hor_max, param_.kine_cons.acc_hor_max,, param_.kine_cons.acc_ver_max);

        u.velocity = Kp.asDiagonal() * (des.p - odom.p);

        Eigen::Vector3d vel_err = u.velocity - odom.v;
        u.velocity = odom.v + vel_err;
        // u.velocity(0) = std::min(u.velocity(0), param_.kine_cons.vel_hor_max);
        // u.velocity(1) = std::min(u.velocity(1), param_.kine_cons.vel_hor_max);
        // u.velocity(2) = std::min(u.velocity(2), param_.kine_cons.vel_ver_max);

        u.yaw = des.yaw;

        debug_msg_.des_p_x = des.p(0);
        debug_msg_.des_p_y = des.p(1);
        debug_msg_.des_p_z = des.p(2);
        
        debug_msg_.des_v_x = (des.p - odom.p)(0);
        debug_msg_.des_v_y = (des.p - odom.p)(1);
        debug_msg_.des_v_z = (des.p - odom.p)(2);

        debug_msg_.cmd_v_x = u.velocity(0);
        debug_msg_.cmd_v_y = u.velocity(1);
        debug_msg_.cmd_v_z = u.velocity(2);
        
        debug_msg_.des_yaw = u.yaw;

        return debug_msg_;
    }

    void Position_Control::init(ctrl_node::Parameter_t &param){
        param_ = param;
    }

    quadrotor_msgs::Px4ctrlDebug Position_Control::calculateControl(const Desired_State_t &des, 
                                                                    const ctrl_node::Odom_Data_t &odom, 
                                                                    VP_Controller_Output_t &u){
    // 

    }
}