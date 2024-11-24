#include "planner.hpp"

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

        // check validation of route_list
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

    void Planner::qr_pose_callback(const geometry_msgs::Pose::ConstPtr& msg)
    {
        Eigen::Vector3d pose(msg->position.x, msg->position.y, msg->position.z);
        landInfo.qr_detected_pose.emplace_back(pose);
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

        quadrotor_msgs::PositionCommand cmd_msg;

        cmd_msg.header.stamp = ros::Time::now();
        cmd_msg.position.x = current_odom(0) + pos_err(0) ;
        cmd_msg.position.y = current_odom(1) + pos_err(1) ;
        cmd_msg.position.z = current_odom(2) + pos_err(2) ;

        cmd_msg.yaw = current_odom(3) + yaw_err;

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

            // ros::Duration(0.5).sleep(); //Wait the controller stable
            get_err(current_route, current_odom, pos_err, yaw_err);

            printf("current_target[x, y, z, yaw]: [%f, %f, %f, %f]\n", 
                    route_list[0][current_route], route_list[1][current_route], route_list[2][current_route], route_list[3][current_route]);
        }

        // local planning
        publish_cmd(current_odom, pos_err, yaw_err);

        return true;
    }

    bool Planner::landing_process(){
        double deadLine = 5;
        double timeWindow = (ros::Time::now() - landInfo.start_time).toSec();

        // 1. 找落点
        if(timeWindow > deadLine && !landInfo.qr_pos_found)
        {
            landInfo.qr_pos_found = true;
            landInfo.target_pos[0] = odom_msg.pose.pose.position.x;
            landInfo.target_pos[1] = odom_msg.pose.pose.position.y;
            landInfo.target_pos[2] = 0;
        }
        else{
            if(landInfo.qr_detected_pose.size()>20 && !landInfo.qr_pos_found)
            {
                //1.计算平均值
                Eigen::Vector3d avgPose(0.0, 0.0, 0.0); 
                for(const auto& pose:landInfo.qr_detected_pose)avgPose += pose;  
                avgPose /= landInfo.qr_detected_pose.size();

                //2.找最大值
                double maxDistance = 0;
                uint32_t index = 0;
                for (size_t i = 0; i < landInfo.qr_detected_pose.size(); ++i)
                {

                    double distance = (landInfo.qr_detected_pose[i] - avgPose).norm();
                    if(distance>maxDistance)
                    {
                        maxDistance = distance;
                        index = i;
                    }
                }

                //3.足够精确则确定落点 
                if(maxDistance<0.1){
                    landInfo.qr_pos_found = true;
                    landInfo.target_pos = avgPose;

                }// 否则剔除离群点
                else{
                    landInfo.qr_detected_pose.erase(landInfo.qr_detected_pose.begin() + index); 
                }
                

            }
        }

        // 2. 没找到直接退出
        if(!landInfo.qr_pos_found)return true;

        // 3. 降落
        Eigen::Quaterniond ori;
        ori.w() = odom_msg.pose.pose.orientation.w;
        ori.x() = odom_msg.pose.pose.orientation.x;
        ori.y() = odom_msg.pose.pose.orientation.y;
        ori.z() = odom_msg.pose.pose.orientation.z;
        double current_yaw = atan2(2.0*(ori.x()*ori.y() + ori.w()*ori.z()), 1.0 - 2.0 * (ori.y() * ori.y() + ori.z() * ori.z()));
        Eigen::Vector4d current_odom(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z, current_yaw);
        Eigen::Vector3d pos_err;
        double yaw_err;
        Eigen::Vector4d err;
        err.segment(0,3) = landInfo.target_pos;
        err[3] = 0;
        err = err - current_odom;
        pos_err = err.segment(0,3);
        yaw_err = err[3];
        publish_cmd(current_odom, pos_err, yaw_err);

        if(pos_err.norm() < target_pos_inflation && std::abs(yaw_err) < target_yaw_inflation)
        {
            return false;
        }
        return true;
        
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
                    plan_state = LAND; // change to LAND mode
                    landInfo.start_time = ros::Time::now();
                    landInfo.qr_detected_pose.clear();
                    ROS_INFO("\033[32m[planner]: Route following mission completed!\033[32m");
                }
                break;

            case LAND:
                if(!landing_process())
                {
                    plan_state = STANDBY;
                    ctrl_ready_trigger = false;
                    ROS_INFO("\033[32m[planner]: Landing mission completed!\033[32m");      
                }
                break;
            
            default:
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
    planner.qr_sub = nh.subscribe<geometry_msgs::Pose>("/qr/pose", 1, boost::bind(&planning::Planner::qr_pose_callback, &planner, _1),
                                                        ros::VoidConstPtr(),
                                                        ros::TransportHints().tcpNoDelay());

    planner.ctrl_trigger_sub = nh.subscribe<geometry_msgs::PoseStamped>("/traj_start_trigger", 1, 
                                                                        boost::bind(&planning::Planner::ctrl_trigger_callback, &planner, _1),
                                                                        ros::VoidConstPtr(),
                                                                        ros::TransportHints().tcpNoDelay());
    
    planner.cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("cmd", 10);

    ros::Rate r(planner.planner_fre);
    while(ros::ok()){
        r.sleep();
        ros::spinOnce();
        planner.main_loop();
    }
}