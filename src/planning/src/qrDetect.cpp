#include "qrDetect.hpp"
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <quadrotor_msgs/PositionCommand.h>


int main(int argc, char *argv[]){
    ros::init(argc, argv, "qrDetect");
    ros::NodeHandle nh("~");

    ros::Rate r(10);
    while(ros::ok()){
        r.sleep();
        std::cout << "hello tag!" << std::endl;
        ros::spinOnce();
    }
}