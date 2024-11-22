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
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    ros::Subscriber cam_down_sub;
    std::vector<std::pair<int, cv::Mat>> saved_markers;
    std::vector<char> occupied_ids;

public:
    QrDetect(ros::NodeHandle& nh)
        : occupied_ids(250,0)
    {
        dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
        cam_down_sub = nh.subscribe("/cam_down/image_raw", 1, &QrDetect::camDownCallback, this);
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

            cv::aruco::drawDetectedMarkers(image, marker_corners, marker_ids);

            // 3. 
            for (size_t i = 0; i < marker_ids.size(); ++i) {
                std::vector<cv::Point2f> corners = marker_corners[i];
                cv::Rect bounding_box = cv::boundingRect(corners);
                cv::Mat cropped_marker = image(bounding_box).clone();

                if(occupied_ids[marker_ids[i]] < 5)
                {
                    if(++occupied_ids[marker_ids[i]]==5)
                    {
                        saved_markers.emplace_back(marker_ids[i], cropped_marker);
                        std::string win_name = "marker"+std::to_string(marker_ids[i]);
                        cv::resize(cropped_marker,cropped_marker,cv::Size(200,200));
                        cv::imshow(win_name, cropped_marker);
                    }
                }

            }
        }

        // 显示图像
        cv::imshow("ArUco Detection", image);
        std::cout <<"all marker:"<< saved_markers.size() << std::endl;
        cv::waitKey(1); // 处理图像
    }
    
};
