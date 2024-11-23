#pragma once
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>


class QrDetect
{
private:

    const uchar MIN_COUNT=5;
    // camera
    ros::Subscriber sub_cam_down;
    cv::Mat cam_down_matrix_,cam_front_matrix_;
    cv::Mat cam_down_coeff_,cam_front_coeff_;
    
    // aruco
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    std::vector<std::pair<int, cv::Mat>> saved_markers;
    std::vector<char> occupied_ids;

public:
    QrDetect(ros::NodeHandle& nh)
        : occupied_ids(250,0)
    {
        // 1. dwon camera info
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
        std::cout << cam_down_matrix_ << std::endl;
        std::cout << cam_down_coeff_ << std::endl;
        // 2. front camera info

        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        sub_cam_down = nh.subscribe("/cam_down/image_raw", 1, &QrDetect::camDownCallback, this);
        ROS_INFO("...QR detect running...");
    }

    void mainLoop(void)
    {
        // 1.show images
        for(auto marker:saved_markers)
        {
            cv::Mat cropped_marker;
            std::string win_name = "marker"+std::to_string(marker.first);
            cv::resize(marker.second,cropped_marker,cv::Size(200,200));
            cv::imshow(win_name, cropped_marker);
            cv::waitKey(1);
        }
    }

    void camDownCallback(const sensor_msgs::Image::ConstPtr& msg) 
    {    
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat image = cv_ptr->image.clone();
        detectArUco(image);
    }

    void detectArUco(cv::Mat& image) {
        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;

        // 1.detect
        cv::aruco::detectMarkers(image, dictionary, marker_corners, marker_ids);

        // 2.
        if (!marker_ids.empty()) {

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);
            std::cout << cam_down_matrix_ << std::endl;
            std::cout << cam_down_coeff_ << std::endl;
            cv::aruco::estimatePoseSingleMarkers(marker_corners, 0.16, cam_down_matrix_, cam_down_coeff_, rvecs, tvecs);
            std::cout<<"R :"<<rvecs[0]<<std::endl;
            std::cout<<"T :"<<tvecs[0]<<std::endl;
            // 3. 
            for (size_t i = 0; i < marker_ids.size(); ++i) {
                std::vector<cv::Point2f> corners = marker_corners[i];
                cv::Rect bounding_box = cv::boundingRect(corners);
                cv::Mat cropped_marker = image(bounding_box).clone();

                
               // cv::aruco::estimatePoseSingleMarkers(marker_corners[i], 0.05, cam_down_matrix_, cam_down_coeff_, rvecs, tvecs);
                cv::aruco::drawAxis(image, cam_down_matrix_, cam_down_coeff_, rvecs[i], tvecs[i], 0.1);

                if(occupied_ids[marker_ids[i]] < MIN_COUNT)
                {
                    if(++occupied_ids[marker_ids[i]]==MIN_COUNT)
                    {
                        saved_markers.emplace_back(marker_ids[i], cropped_marker);
                        ROS_INFO("NEW TAG ID:%d",marker_ids[i]);
                    }
                }

            }
        }

        cv::imshow("ArUco Detection", image);
        // std::cout <<"all marker:"<< saved_markers.size() << std::endl;
        cv::waitKey(1); 
    }
    
};
