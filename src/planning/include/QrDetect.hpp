#pragma once
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>

class QrDetect
{
private:
    
    // px4
    nav_msgs::Odometry odom_msg;
    bool odom_received = false;

    // camera
    ros::Subscriber sub_cam_front,sub_cam_down,sub_odom;
    ros::Publisher pub_qr_pose;
    cv::Mat cam_down_matrix_,cam_front_matrix_;
    cv::Mat cam_down_coeff_,cam_front_coeff_;
    
    // aruco
    struct QrInfo{
        int id;
        double time;
        Eigen::Matrix4d T;
        uint32_t pubCnt = 0;
        bool isOnGround = 0;
    };
    const uchar MIN_COUNT=5;
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector<std::pair<QrInfo, cv::Mat>> saved_markers;
    
    std::map<int,std::vector<QrInfo>> map_markers;

public:
    QrDetect(ros::NodeHandle& nh)
    {
        // 1.camera_down info
        std::vector<double> cameraMatrixVec;
        std::vector<double> distCoeffsVec;
        if (!nh.getParam("cam_down_matrix", cameraMatrixVec) || cameraMatrixVec.size() != 9) {
            ROS_ERROR("Failed to get cam_down_matrix or incorrect size");
            return;
        }
        if (!nh.getParam("cam_down_coeff", distCoeffsVec)) {
            ROS_ERROR("Failed to get cam_down_coeff");
            return;
        }
        cam_down_matrix_ = cv::Mat(cameraMatrixVec).reshape(0, 3).clone(); 
        cam_down_coeff_ = cv::Mat(distCoeffsVec).clone().t(); 

        // 2. front camera info
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

        // 3. ros pub sub
        sub_odom = nh.subscribe("/mavros/local_position/odom", 1,&QrDetect::odomCallback, this,ros::TransportHints().tcpNoDelay());
        ros::Duration(1).sleep();
        // ROS_INFO("qrDetect waiting for odometry message...");
        // while (ros::ok() && !odom_received) {
        //     ros::spinOnce();             
        //     ros::Duration(0.5).sleep();
        //     std::cout << ".";std::cout.flush();
        // }
        odom_msg.pose.pose.orientation.w = 1;
        odom_msg.pose.pose.orientation.x = 0;
        odom_msg.pose.pose.orientation.y = 0;
        odom_msg.pose.pose.orientation.z = 0;
        odom_msg.pose.pose.position.x = 0;
        odom_msg.pose.pose.position.y = 0;
        odom_msg.pose.pose.position.z = 0;

        ROS_INFO("Odometry message received!");

        sub_cam_down = nh.subscribe("/cam_down/image_raw", 1, &QrDetect::camDownCallback, this,ros::TransportHints().tcpNoDelay());
        sub_cam_front = nh.subscribe("/cam_front/image_raw", 1, &QrDetect::camFrontCallback, this,ros::TransportHints().tcpNoDelay());
        pub_qr_pose = nh.advertise<geometry_msgs::Pose>("/qr/pose", 5);
    }
    
    void mainLoop(void)
    {

        // 1.show images
        for(auto marker:saved_markers)
        {
            QrInfo qr = marker.first;
            cv::Mat image = marker.second;
            if(qr.isOnGround)
            {
                if(qr.pubCnt<5)
                {
                    qrPosePub(qr.T);
                    qr.pubCnt++;
                }
                continue;
            }
            
            cv::Mat cropped_marker;
            std::string win_name = "marker"+std::to_string(marker.first.id);
            cv::resize(image,cropped_marker,cv::Size(200,200));
            cv::imshow(win_name, cropped_marker);
            cv::waitKey(1);
        }

    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_msg = *msg;  
        odom_received = true;
    }
    void camFrontCallback(const sensor_msgs::Image::ConstPtr& msg) 
    {    
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        detectArUco(image);
    }
    void camDownCallback(const sensor_msgs::Image::ConstPtr& msg) 
    {    
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image;
        detectArUco(image);
    }
    void detectArUco(cv::Mat& image) {
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        std::vector<cv::Vec3d> rvecs, tvecs;
        double time_now = ros::Time::now().toSec();
        // 1. detect
        cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids);
        cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.16, cam_down_matrix_, cam_down_coeff_, rvecs, tvecs);

        // 2. 
        if (!marker_ids.empty()) {
            for (size_t i = 0; i < marker_ids.size(); ++i) {
                cv::aruco::drawAxis(image, cam_down_matrix_, cam_down_coeff_, rvecs[i], tvecs[i], 0.1);
                // 二维码在world下的位姿
                cv::Mat R_cv;
                cv::Rodrigues(rvecs[i], R_cv);  
                Eigen::Matrix3d R_eigen;
                R_eigen = Eigen::Map<Eigen::Matrix3d>(R_cv.ptr<double>(), 3, 3).transpose();
                Eigen::Vector3d t_eigen;
                t_eigen << tvecs[i][0], tvecs[i][1], tvecs[i][2];
                Eigen::Matrix4d T = Eigen::Matrix4d::Identity();  
                T.block<3, 3>(0, 0) = R_eigen;                    
                T.block<3, 1>(0, 3) = t_eigen;   
                Eigen::Matrix4d marker_in_world = cam2world(T);
                QrInfo qrInfo;
                qrInfo.id = marker_ids[i];
                qrInfo.time = time_now;
                qrInfo.T = marker_in_world;
                if(abs(marker_in_world(2,2))>0.86)qrInfo.isOnGround=true;

                // 0. 截图
                std::vector<cv::Point2f> corners = marker_corners[i];
                cv::Rect bounding_box = cv::boundingRect(corners);
                cv::Mat cropped_marker = image(bounding_box).clone();
                // 1. 误检测判断
                int id = marker_ids[i];
                bool isValid = 0;
                if(map_markers[id].empty())isValid = 1;
                else if((time_now - map_markers[id].back().time)<0.1)isValid = 1;

                // 2. 
                if(map_markers[id].size()<MIN_COUNT){
                    if(isValid){
                        map_markers[id].emplace_back(qrInfo);
                    }
                    else{
                        map_markers.erase(id);
                    }
                    // 在1s内连续检测成功10次 保存二维码：ID，坐标，图像
                    if(map_markers[id].size()==MIN_COUNT){
                        Eigen::Vector3d t;t.setZero();
                        Eigen::Matrix3d R;
                        Eigen::Quaterniond avgR(qrInfo.T.block<3,3>(0,0));
                        for(auto it:map_markers[id])
                        {
                            t += it.T.block<3,1>(0,3);
                            Eigen::Quaterniond temp(it.T.block<3,3>(0,0));
                            avgR.coeffs() += temp.coeffs();
                        }
                        t /= map_markers[id].size();
                        R = avgR.normalized().toRotationMatrix();
                        qrInfo.T.block<3,3>(0,0) = R;
                        qrInfo.T.block<3,1>(0,3) = t;

                        if(abs(R(2,2))>0.86) //30 deg
                        {
                            qrInfo.isOnGround = true;
                            qrInfo.T.block<3,1>(0,3) *= (5/2); //修正距离
                            qrInfo.T(3,2) = 2;
                        }
                        else qrInfo.isOnGround = false;
                        saved_markers.emplace_back(qrInfo,cropped_marker);
                        ROS_INFO("NEW TAG ID:%d",marker_ids[i]);
                    }
                }

            }
        }

        cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
        cv::imshow("ArUco Detection", image);
        cv::waitKey(1); 
    }
    void qrPosePub(const Eigen::Matrix4d &pose)
    {
        geometry_msgs::Pose pose_msg;
        Eigen::Quaterniond R(pose.block<3, 3>(0, 0));
        Eigen::Vector3d t = pose.block<3, 1>(0, 3);

        pose_msg.position.x = t[0]; 
        pose_msg.position.y = t[1];
        pose_msg.position.z = t[2];

        pose_msg.orientation.x = R.x();
        pose_msg.orientation.y = R.y();
        pose_msg.orientation.z = R.z();
        pose_msg.orientation.w = R.w();

        pub_qr_pose.publish(pose_msg);

    }
    Eigen::Matrix4d cam2world(const Eigen::Matrix4d &pose)
    {
        Eigen::Matrix4d res;
        //1.cam to body
        Eigen::Matrix4d T_body_cam = Eigen::Matrix4d::Zero();
        T_body_cam.block<3, 3>(0, 0) << 0,  0, 1,
                                        -1, 0, 0,
                                        0, -1, 0;
        T_body_cam.block<4,1>(0,3)  << 0.1, 0, 0.05,1;
        res = T_body_cam*pose;

        //2.body to world
        Eigen::Matrix4d T_world_body = Eigen::Matrix4d::Zero();
        Eigen::Quaterniond R(
            odom_msg.pose.pose.orientation.w,  
            odom_msg.pose.pose.orientation.x,  
            odom_msg.pose.pose.orientation.y,  
            odom_msg.pose.pose.orientation.z 
        );
        T_world_body.block<3,3>(0,0) = R.toRotationMatrix();
        T_world_body.block<4,1>(0,3) <<  odom_msg.pose.pose.position.x,
                                         odom_msg.pose.pose.position.y,
                                         odom_msg.pose.pose.position.z,
                                         1;
        res = T_world_body * res;
        return res;
    }
    Eigen::Matrix4d cam_down2world(const Eigen::Matrix4d &pose)
    {
        Eigen::Matrix4d res;
        //1.cam to body
        Eigen::Matrix4d T_body_cam = Eigen::Matrix4d::Zero();
        T_body_cam.block<3, 3>(0, 0) << 0,  0, 1,
                                        -1, 0, 0,
                                        0, -1, 0;
        T_body_cam.block<4,1>(0,3)  << 0.1, 0, 0.05,1;
        res = T_body_cam*pose;

        //2.body to world
        Eigen::Matrix4d T_world_body = Eigen::Matrix4d::Zero();
        Eigen::Quaterniond R(
            odom_msg.pose.pose.orientation.w,  
            odom_msg.pose.pose.orientation.x,  
            odom_msg.pose.pose.orientation.y,  
            odom_msg.pose.pose.orientation.z 
        );
        T_world_body.block<3,3>(0,0) = R.toRotationMatrix();
        T_world_body.block<4,1>(0,3) <<  odom_msg.pose.pose.position.x,
                                         odom_msg.pose.pose.position.y,
                                         odom_msg.pose.pose.position.z,
                                         1;
        res = T_world_body * res;
        return res;
    }
};
