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

        ros::Publisher aruco_id_pub;//发布二维码ID

        ros::Publisher aruco_img_pub;//发布二维码图像

        Planner(const ros::NodeHandle &nh);

        void odom_callback(const nav_msgs::OdometryConstPtr &msg);

        void ctrl_trigger_callback(const geometry_msgs::PoseStampedConstPtr &msg);

        double get_min_duration(const Eigen::Vector3d& position_err, const double& yaw_err);

        void get_err(const int current_route, const Eigen::Vector4d& current_odom, Eigen::Vector3d& pos_err, double& yaw_err);

        void publish_cmd(const Eigen::Vector4d& current_odom, const Eigen::Vector3d& pos_err, const double& yaw_err);

        bool route_follow_process();

        bool landing_process();

        Eigen::Vector3d calculate_position_error(const Eigen::Vector3d& current_position, const Eigen::Vector3d& target_position);

        bool detect_aruco_marker();//通过摄像头检测是否存在ArUco标识码

        bool get_aruco_position(Eigen::Vector3d& position);//将标识码的像素坐标转换为相对于无人机的三维坐标

        void main_loop();

        void detect_aruco();

        bool detect_and_get_aruco(Eigen::Vector3d& marker_position);
        
        bool detect_and_get_aruco(cv::Mat& image, std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners);


        cv::Mat get_camera_frame();

        std::string save_path_ = "/tmp/aruco_data/";
    };
}

#endif