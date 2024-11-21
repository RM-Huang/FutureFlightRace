#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <quadrotor_msgs/Px4ctrlDebug.h>

#include "input.hpp"
#include "param.hpp"

namespace Controller{

    template <typename Scalar_t>
    Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(const Eigen::Quaternion<Scalar_t>& q_) {
        Eigen::Quaternion<Scalar_t> q = q_.normalized();

        Eigen::Matrix<Scalar_t, 3, 1> ypr;
        ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()), 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
        ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
        ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

        return ypr;
    }

    struct Desired_State_t{
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Vector3d a;
        Eigen::Vector3d j;
        Eigen::Quaterniond q;
        Eigen::Vector3d omg;
        double yaw;
        double yaw_rate;

        Desired_State_t(){};

        Desired_State_t(ctrl_node::Odom_Data_t &odom):
            p(odom.p),
            v(Eigen::Vector3d::Zero()),
            a(Eigen::Vector3d::Zero()),
            j(Eigen::Vector3d::Zero()),
            q(odom.q),
            omg(Eigen::Vector3d::Zero()),
            yaw(quaternion_to_ypr(odom.q)(0)),
            yaw_rate(0){};
    };

    struct VP_Controller_Output_t{
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;

        double yaw;
    };

    class Position_Control{
        private:
        ctrl_node::Parameter_t param_;
        quadrotor_msgs::Px4ctrlDebug debug_msg_;

        public:
        void init(ctrl_node::Parameter_t &param);
        quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des, 
                                                      const ctrl_node::Odom_Data_t &odom, 
                                                      VP_Controller_Output_t &u);
    };

    class Velocity_Control{
        private:
        ctrl_node::Parameter_t param_;
        quadrotor_msgs::Px4ctrlDebug debug_msg_;

        public:
        void init(ctrl_node::Parameter_t &param);
        quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des, 
                                                      const ctrl_node::Odom_Data_t &odom, 
                                                      VP_Controller_Output_t &u);
    };
}

#endif