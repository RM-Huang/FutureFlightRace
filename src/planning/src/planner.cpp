#include "planner.hpp"
#include <opencv2/aruco.hpp>

namespace planning{
    Planner::Planner(const ros::NodeHandle &nh){
        route_list.resize(4);
        
        nh.getParam("planner_fre", planner_fre);
        nh.getParam("route_list_x", route_list[0]);
        nh.getParam("route_list_y", route_list[1]);
        nh.getParam("route_list_z", route_list[2]);
        nh.getParam("route_list_yaw", route_list[3]);
        nh.getParam("max_vel_hor", MAX_VEL_HOR);
        nh.getParam("max_vel_ver", MAX_VEL_VER);
        nh.getParam("max_omega", MAX_OMEGA);
        nh.getParam("target_pos_inflation", target_pos_inflation);
        nh.getParam("target_yaw_inflation", target_yaw_inflation);
        nh.getParam("aruco_save_path", save_path_);//在launch文件中设置保存路径

        // 检查维度是否一致
        if((route_list[0].size() ^ route_list[1].size() ^ route_list[2].size() ^ route_list[3].size())){
            while(1){
                ROS_ERROR("[planner]:Route list have invalid num in dimensions!");
                ros::Duration(1.0).sleep();
            }
        }else{
            std::cout << "route_list_size = " << route_list[0].size() << std::endl;
        }

        plan_state = STANDBY;
        ROS_INFO("\033[32m[planner]: Planner init succeed!\033[32m");
    }

    void Planner::odom_callback(const nav_msgs::OdometryConstPtr &msg){
        odom_msg = *msg;
    }

    void Planner::ctrl_trigger_callback(const geometry_msgs::PoseStampedConstPtr &msg){
        ctrl_ready_trigger = true;
        if(!ctrl_ready_trigger){
           ctrl_ready_trigger = true; 
           ROS_INFO("\033[32m[planner]: Ctrl trigger received!\033[32m");
        }
    }

    double Planner::get_min_duration(const Eigen::Vector3d& position_err, const double& yaw_err){
        double dur = std::max(std::abs(position_err[0] / MAX_VEL_HOR), std::abs(position_err[1] / MAX_VEL_HOR));
        dur = std::max(dur, std::abs(position_err[2] / MAX_VEL_VER));
        return std::max(dur, std::abs(yaw_err / MAX_OMEGA));
    }

    void Planner::get_err(const int current_route, const Eigen::Vector4d& current_odom, Eigen::Vector3d& pos_err, double& yaw_err){
        Eigen::Vector4d err(route_list[0][current_route], route_list[1][current_route], route_list[2][current_route], route_list[3][current_route]);
        err = err - current_odom;

        pos_err = err.segment(0,3);
        yaw_err = err[3];
    }

    void Planner::publish_cmd(const Eigen::Vector4d& current_odom, const Eigen::Vector3d& pos_err, const double& yaw_err){
        double local_planning_dur = get_min_duration(pos_err, yaw_err);

        quadrotor_msgs::PositionCommand cmd_msg;

        cmd_msg.header.stamp = ros::Time::now();
        cmd_msg.position.x = current_odom(0) + pos_err(0) / local_planning_dur;
        cmd_msg.position.y = current_odom(1) + pos_err(1) / local_planning_dur;
        cmd_msg.position.z = current_odom(2) + pos_err(2) / local_planning_dur;

        cmd_msg.yaw = current_odom(3) + yaw_err / local_planning_dur;

        cmd_pub.publish(cmd_msg);
    }

    bool Planner::route_follow_process(){
        
        Eigen::Quaterniond ori;

        ori.w() = odom_msg.pose.pose.orientation.w;
        ori.x() = odom_msg.pose.pose.orientation.x;
        ori.y() = odom_msg.pose.pose.orientation.y;
        ori.z() = odom_msg.pose.pose.orientation.z;
        double current_yaw = atan2(2.0*(ori.x()*ori.y() + ori.w()*ori.z()), 1.0 - 2.0 * (ori.y() * ori.y() + ori.z() * ori.z()));

        Eigen::Vector4d current_odom(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z, current_yaw);
        Eigen::Vector3d pos_err;
        double yaw_err;

        get_err(current_route, current_odom, pos_err, yaw_err);

        if(pos_err.norm() < target_pos_inflation && std::abs(yaw_err) < target_yaw_inflation){
            current_route += 1;
            std::cout << "current_route = " << current_route << std::endl;
            if(current_route >= route_list[0].size()){
                return false;
            }

            ros::Duration(0.5).sleep(); //等待稳定
            get_err(current_route, current_odom, pos_err, yaw_err);

            printf("current_target[x, y, z, yaw]: [%f, %f, %f, %f]\n", 
                    route_list[0][current_route], route_list[1][current_route], route_list[2][current_route], route_list[3][current_route]);
        }

        // local planning
        publish_cmd(current_odom, pos_err, yaw_err);

        return true;
    }

    bool Planner::landing_process(){
        // 检测aruco二维码
        Eigen::Vector3d marker_position;
        if (!detect_and_get_aruco(marker_position)) {
            ROS_WARN("[planner]: ArUco marker not detected. Hovering.");
            ros::Duration(0.5).sleep(); // 等待
            return false;
        }

        // 计算位置误差
        Eigen::Vector3d current_position(odom_msg.pose.pose.position.x,
                                        odom_msg.pose.pose.position.y,
                                        odom_msg.pose.pose.position.z);
        Eigen::Vector3d position_error = calculate_position_error(current_position, marker_position);

        // 检测无人机是否接近
        if (position_error.norm() < target_pos_inflation) {
            if (odom_msg.pose.pose.position.z <= 0.1) { // 准备降落
                ROS_INFO("\033[32m[planner]: Landing completed.\033[32m");
                return true; // 完成着陆
            } else {
                // 逐步降落
                Eigen::Vector3d descent_position = marker_position;
                descent_position[2] = current_position[2] - 0.1; // 每次0，1m
                publish_cmd(Eigen::Vector4d(descent_position[0], descent_position[1], descent_position[2], 0.0),
                            position_error, 0.0);
            }
        } else {
            // 调整水平误差
            Eigen::Vector3d adjusted_position = current_position + position_error;
            publish_cmd(Eigen::Vector4d(adjusted_position[0], adjusted_position[1], current_position[2], 0.0),
                        position_error, 0.0);
        }

        return false; //继续等待着陆

    }

    Eigen::Vector3d Planner::calculate_position_error(const Eigen::Vector3d& current_position, const Eigen::Vector3d& target_position) {
        return target_position - current_position;
    }

    void ensure_directory_exists(const std::string& directory) {
        struct stat info;

        // 检测路径是否存在
        if (stat(directory.c_str(), &info) != 0) {
            // 若不存在，创建一个路径
            if (mkdir(directory.c_str(), 0777) != 0) {
                ROS_ERROR("[planner]: Failed to create directory: %s", directory.c_str());
            } else {
                ROS_INFO("[planner]: Directory created: %s", directory.c_str());
            }
        } else if (!(info.st_mode & S_IFDIR)) {
            // 错误信息
            ROS_ERROR("[planner]: Path exists but is not a directory: %s", directory.c_str());
        }
    }


    void Planner::detect_aruco() {
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;
    cv::Mat aruco_image;

    if (detect_and_get_aruco(aruco_image, marker_ids, marker_corners)) {
        ROS_INFO("\033[32m[planner]: ArUco marker detected!\033[32m");

        for (size_t i = 0; i < marker_ids.size(); ++i) {
            int id = marker_ids[i];
            ROS_INFO("[planner]: Detected ArUco ID: %d", id);

            // 打印id
            std_msgs::Int32 id_msg;
            id_msg.data = id;
            aruco_id_pub.publish(id_msg);

            // 输出到ros话题
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", aruco_image).toImageMsg();
            aruco_img_pub.publish(img_msg);
            ROS_INFO("[planner]: Published ArUco ID and image for marker %d.", id);

            // 保存图像和id
            save_aruco_data(id, aruco_image);
        }
        } else {
        ROS_WARN("[planner]: No ArUco markers detected at this path point.");
        }
        }

    void Planner::save_aruco_data(int id, const cv::Mat& image) {
        // 确保路径存在
        ensure_directory_exists(save_path_);

        // 分配时间戳
        auto now = std::chrono::system_clock::now();
        auto now_time = std::chrono::system_clock::to_time_t(now);
        std::ostringstream timestamp_stream;
        timestamp_stream << std::put_time(std::localtime(&now_time), "%Y%m%d_%H%M%S");
        std::string timestamp = timestamp_stream.str();

        // 分配特殊文件名
        std::string image_filename = save_path_ + "aruco_marker_" + std::to_string(id) + "_" + timestamp + ".png";
        std::string log_filename = save_path_ + "aruco_log.txt";

        // 保存
        if (!cv::imwrite(image_filename, image)) {
            ROS_ERROR("[planner]: Failed to save image to %s", image_filename.c_str());
            return;
        }
        ROS_INFO("[planner]: Saved ArUco image to %s", image_filename.c_str());

        // 保存日志
        std::ofstream log_file(log_filename, std::ios::app);
        if (log_file.is_open()) {
            log_file << "ID: " << id << ", Timestamp: " << timestamp << "\n";
            log_file.close();
            ROS_INFO("[planner]: Saved ArUco ID %d to log file: %s", id, log_filename.c_str());
        } else {
            ROS_ERROR("[planner]: Failed to open log file for writing: %s", log_filename.c_str());
        }
    }

    bool Planner::detect_and_get_aruco(Eigen::Vector3d& marker_position) {
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::Mat image;

    if (!detect_and_get_aruco(image, ids, corners)) {
        return false;
    }

    // 检测到图像
    if (!ids.empty() && !corners.empty()) {
        // Convert pixel coordinates to 3D coordinates (simplified for example)
        marker_position = Eigen::Vector3d(corners[0][0].x, corners[0][0].y, 0.0);
        return true;
    }

    return false;
    }

    bool planning::Planner::detect_and_get_aruco(cv::Mat& image, std::vector<int>& ids, std::vector<std::vector<cv::Point2f>>& corners) {
    // 实现 ArUco 检测逻辑，调用 OpenCV 的 ArUco 检测函数。
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::detectMarkers(image, dictionary, corners, ids);

    return !ids.empty(); // 如果检测到至少一个标记，则返回 true
    }

cv::Mat Planner::get_camera_frame() {
    cv::VideoCapture cap(0); // 打开相机
    if (!cap.isOpened()) {
        ROS_ERROR("[planner]: Unable to open camera!");
        return cv::Mat();
    }

    cv::Mat frame;
    cap >> frame;
    cap.release();
    return frame;
}

    void Planner::main_loop(){
        switch (plan_state)
        {
            case STANDBY:
                if(ctrl_ready_trigger){
                    ros::Duration(1.0).sleep();
                    plan_state = FOLLOW;
                    ROS_INFO("\033[32m[planner]: Start route following!\033[32m");
                }
                break;
            
            case FOLLOW:
                if(!route_follow_process()){
                    ros::Duration(0.5).sleep(); // Allow state transition stabilization
                    plan_state = LAND; // change to LAND mode
                    ctrl_ready_trigger = false;
                    ROS_INFO("\033[32m[planner]: Route following mission completed!\033[32m");
                }
                else{

                    detect_aruco();

                }
                break;

            case LAND:
                if (landing_process()){
                    ros::Duration(0.5).sleep(); // Stabilize after landing
                    plan_state = STANDBY;
                }
                break;
            default:
                ROS_WARN("[planner]: Unknown state in main_loop: %d. Resetting to STANDBY.", plan_state);
                    plan_state = STANDBY;
                break;
        }
    }
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh("~");

    planning::Planner planner(nh);

    planner.odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, boost::bind(&planning::Planner::odom_callback, &planner, _1),
                                                        ros::VoidConstPtr(),
                                                        ros::TransportHints().tcpNoDelay());

    planner.ctrl_trigger_sub = nh.subscribe<geometry_msgs::PoseStamped>("/traj_start_trigger", 1, 
                                                                        boost::bind(&planning::Planner::ctrl_trigger_callback, &planner, _1),
                                                                        ros::VoidConstPtr(),
                                                                        ros::TransportHints().tcpNoDelay());
    
    planner.cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 5);

    planner.aruco_id_pub = nh.advertise<std_msgs::Int32>("aruco_id",10);//初始化发布二维码id
    planner.aruco_img_pub = nh.advertise<sensor_msgs::Image>("aruco_image",10);//初始化发布二维码图像

    ros::Rate r(planner.planner_fre);
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();
        planner.main_loop();
    }
}