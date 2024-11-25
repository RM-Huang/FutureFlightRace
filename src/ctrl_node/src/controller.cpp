#include "controller.hpp"

namespace Controller{
    static double q2yaw(const Eigen::Quaterniond &ori)
    {
        return atan2(2.0*(ori.x()*ori.y() + ori.w()*ori.z()), 1.0 - 2.0 * (ori.y() * ori.y() + ori.z() * ori.z()));
    }

    void Velocity_Control::init(ctrl_node::Parameter_t &param){
        param_ = param;
        input.vel_last.setZero();
        input.yaw_last = 0;
    }

    double Velocity_Control::get_vel_err(const Desired_State_t &des, const ctrl_node::Odom_Data_t &odom){
        // Eigen::Vector3d des_v = Kp.asDiagonal() * (des.p - odom.p);
        return 0;
    }

    quadrotor_msgs::Px4ctrlDebug Velocity_Control::calculateControl(const Desired_State_t &des, 
                                                                    const ctrl_node::Odom_Data_t &odom, 
                                                                    VP_Controller_Output_t &u){
        // 
        Eigen::Vector3d Kp(param_.gain.Kvp0, param_.gain.Kvp1, param_.gain.Kvp2);
        Eigen::Vector3d Kd(param_.gain.Kvd0, param_.gain.Kvd1, param_.gain.Kvd2);
        Eigen::Vector3d vel_max(param_.kine_cons.vel_hor_max,param_.kine_cons.vel_hor_max,param_.kine_cons.vel_ver_max);
        Eigen::Vector3d acc_max(param_.kine_cons.acc_hor_max, param_.kine_cons.acc_hor_max, param_.kine_cons.acc_ver_max);
        
        //PD control
        Eigen::Vector3d pos_err(des.p - odom.p);
        u.velocity = Kp.asDiagonal() * pos_err + Kd.asDiagonal()*(pos_err - odom_last_.p);
        odom_last_.p = pos_err;

        //limit vel
        u.velocity = u.velocity.cwiseMax(-vel_max).cwiseMin(vel_max);

        //limit acc
        Eigen::Vector3d vel_err(u.velocity-input.vel_last);
        vel_err = vel_err.cwiseMax(-acc_max).cwiseMin(acc_max);
        u.velocity = input.vel_last + vel_err;
        input.vel_last = u.velocity;
        
        //limit yaw
        double omega_err(des.yaw - input.yaw_last);
        omega_err = omega_err > param_.kine_cons.omega_yaw_max?param_.kine_cons.omega_yaw_max:omega_err;
        omega_err = omega_err < -param_.kine_cons.omega_yaw_max?-param_.kine_cons.omega_yaw_max:omega_err;
        u.yaw =  input.yaw_last + omega_err;
        input.yaw_last = u.yaw;

        //debug
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
        quadrotor_msgs::Px4ctrlDebug data;
        return data;
    }
}