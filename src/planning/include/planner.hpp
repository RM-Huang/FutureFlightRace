#ifndef TRAJ_FOLLOW_HPP
#define TRAJ_FOLLOW_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>

namespace planning{

    class Planner{
        public:

        enum PlanState{
            STANDBY = 0,
            FOLLOW = 1,
            LAND
        };

        int planner_fre = 150;
        double MAX_VEL_HOR = 1.0;
        double MAX_VEL_VER = 1.0;
        double MAX_OMEGA = 0.26; // 15deg
        double target_pos_inflation = 0.3;
        double target_yaw_inflation = 0.1; //6deg

        int current_route = 0;
        bool ctrl_ready_trigger = false;

        PlanState plan_state;
        std::vector<std::vector<double>> route_list; // [x, y, z, yaw]^T

        nav_msgs::Odometry odom_msg;

        ros::Subscriber odom_sub;
        ros::Subscriber ctrl_trigger_sub;
        
        ros::Publisher cmd_pub;

        Planner(const ros::NodeHandle &nh);

        void odom_callback(const nav_msgs::OdometryConstPtr &msg);

        void ctrl_trigger_callback(const geometry_msgs::PoseStampedConstPtr &msg);

        double get_min_duration(const Eigen::Vector3d& position_err, const double& yaw_err);

        void get_err(const int current_route, const Eigen::Vector4d& current_odom, Eigen::Vector3d& pos_err, double& yaw_err);

        bool route_follow_process();

        bool landing_process();

        void main_loop();
    };
}

#endif