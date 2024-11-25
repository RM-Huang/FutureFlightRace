#include "fsm_nodelet.hpp"

namespace ctrl_node{

    void FSM::takeoff_triger_callback(const quadrotor_msgs::TakeoffLandConstPtr& msgPtr){
        if(!takeoff_triger_received){
            takeoff_triger_received = true;
            ROS_INFO("\033[32m[FSM]:Takeoff triger received!\033[32m");
        }else{
            ROS_ERROR("[FSM]:Takeoff triger duplicated!");
        }
    }

    void FSM::publish_trigger(const nav_msgs::Odometry &odom_msg){
        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = "world";
        msg.pose = odom_msg.pose.pose;

        traj_start_triger_pub.publish(msg);
    }

    void FSM::publish_position_ctrl(const Controller::VP_Controller_Output_t &u, const ros::Time &stamp){
        mavros_msgs::PositionTarget msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = std::string("FCU");

        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        msg.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                        mavros_msgs::PositionTarget::IGNORE_VY |
                        mavros_msgs::PositionTarget::IGNORE_VZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        msg.position.x = u.position.x();
        msg.position.y = u.position.y();
        msg.position.z = u.position.z();
        msg.yaw = u.yaw;

        ctrl_pv_pub.publish(msg);
    }

    void FSM::publish_velocity_ctrl(const Controller::VP_Controller_Output_t &u, const ros::Time &stamp){
        mavros_msgs::PositionTarget msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = std::string("FCU");

        msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                        mavros_msgs::PositionTarget::IGNORE_PY |
                        mavros_msgs::PositionTarget::IGNORE_PZ |
                        mavros_msgs::PositionTarget::IGNORE_AFX |
                        mavros_msgs::PositionTarget::IGNORE_AFY |
                        mavros_msgs::PositionTarget::IGNORE_AFZ |
                        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

        msg.velocity.x = u.velocity.x();
        msg.velocity.y = u.velocity.y();
        msg.velocity.z = u.velocity.z();
        msg.yaw = u.yaw;

        ctrl_pv_pub.publish(msg);
    }

    bool FSM::toggle_arm_disarm(bool arm){
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;
        //申请解锁不成功
        if (!(arming_client_srv.call(arm_cmd) && arm_cmd.response.success))
        {
            if (arm)
                ROS_ERROR("ARM rejected by PX4!");
            else
                ROS_ERROR("DISARM rejected by PX4!");

            return false;
        }

        return true;
    }

    void FSM::set_hov_with_odom(){
        hover_pose.head<3>() = odom_data.p;
        hover_pose(3) = Controller::quaternion_to_ypr(odom_data.q)(0); // get yaw
        std::cout<<"hover_pose = "<< odom_data.p.reverse()<<std::endl;

        last_set_hover_pose_time = ros::Time::now();
    }

    void FSM::set_start_pose_for_takeoff(const Odom_Data_t &odom){
        takeoff_state.start_pose.head<3>() = odom.p;
        takeoff_state.start_pose(3) = Controller::quaternion_to_ypr(odom.q)(0); //get yaw

        takeoff_state.toggle_takeoff_time = ros::Time::now();
    }

    Controller::Desired_State_t FSM::get_pv_speed_up_des(const ros::Time& now){
        // double delta_t = (now - takeoff_state.toggle_takeoff_time).toSec();
        // double des_a_z = 0.05;
        // double des_v_z = des_a_z * delta_t;

        Controller::Desired_State_t des;
        // des.p = takeoff_state.start_pose.head<3>() + Eigen::Vector3d(0, 0, param.takeoff_state.height + 0.2);
        des.p = odom_data.p;
        des.v = Eigen::Vector3d::Zero();
        des.a = Eigen::Vector3d::Zero();
        des.j = Eigen::Vector3d::Zero();
        des.yaw = takeoff_state.start_pose(3);
        des.yaw_rate = 0.0;

        return des;
    }

    Controller::Desired_State_t FSM::get_takeoff_des(const Odom_Data_t &odom){
        // double speed = std::max(param.takeoff_state.vel_max, param.takeoff_state.height - odom.p(2));

        Controller::Desired_State_t des;
        des.p << odom.p(0), odom.p(1), takeoff_state.start_pose(2);
        des.p = des.p + Eigen::Vector3d(0, 0, param.takeoff_state.height);
        // des.v = Eigen::Vector3d(0, 0, speed);
        des.v = Eigen::Vector3d::Zero();
        des.a = Eigen::Vector3d::Zero();
        des.j = Eigen::Vector3d::Zero();
        des.yaw = takeoff_state.start_pose(3);
        des.yaw_rate = 0.0;

        return des;
    }

    Controller::Desired_State_t FSM::get_hover_des(){
        Controller::Desired_State_t des;
        des.p = hover_pose.head<3>();
        des.v = Eigen::Vector3d::Zero();
        des.a = Eigen::Vector3d::Zero();
        des.j = Eigen::Vector3d::Zero();
        des.yaw = hover_pose(3);
        des.yaw_rate = 0.0;

        return des;
    }

    Controller::Desired_State_t FSM::get_cmd_des(){
        Controller::Desired_State_t des;
        des.p = cmd_data.p;
        des.v = cmd_data.v;
        des.a = cmd_data.a;
        des.j = cmd_data.j;
        des.omg = cmd_data.omg;
        des.yaw = cmd_data.yaw;
        des.yaw_rate = cmd_data.yaw_rate;

        return des;
    }

    void FSM::fsm_timer(const ros::TimerEvent& event){
        
        ros::Time now_time = ros::Time::now();
        if(!odom_is_received(now_time)){
            ROS_ERROR("[FSM]:No odom! Do Not switch to OFFBOARD mode now!");
            takeoff_triger_received = false;
            current_state = MANUAL;
            ros::Duration(1.0).sleep();
            return;
        }

        // if(state_data.current_state.mode != "OFFBOARD"){ // 若遥控器没切到OFFBOARD模式则退出
        //     takeoff_triger_received = false;
        //     current_state = MANUAL;
        //     return;
        // }

        if(state_data.previous_state.mode != state_data.current_state.mode){ // 若遥控器没切到OFFBOARD模式则退出
            if(state_data.current_state.mode == "OFFBOARD"){
                ROS_INFO("\033[32m[FSM]:Switch to OFFBOARD mode!\033[32m");
            }else if(state_data.previous_state.mode == "OFFBOARD"){
                ROS_INFO("\033[32m[FSM]:Exit OFFBOARD mode! Switch to %s state.\033[32m", state_data.current_state.mode.c_str());
                current_state = MANUAL;
            }
            state_data.previous_state = state_data.current_state;
        }

        Controller::Desired_State_t des(odom_data);
        Controller::VP_Controller_Output_t u;

        switch (current_state){
            case MANUAL:
                if(state_data.current_state.mode == "OFFBOARD"){
                    if(takeoff_triger_received){
                        // Auto_Takeoff conditions check
                        if(state_data.current_state.armed){
                            ROS_ERROR("[FSM]:Reject Auto_Takeoff, vehicle is already armed!");
                        // }else if(odom_data.v.norm() > 0.1){
                        //     ROS_ERROR("[FSM]:Reject Auto_Takeoff, Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data.v.norm());
                        }else{
                            if(!toggle_arm_disarm(true)){ //arming rejected
                                ROS_ERROR("[FSM]:takeoff arming rejected by PX4!");
                            }else{
                                current_state = TAKEOFF;
                                takeoff_state.first_time = true;
                                set_start_pose_for_takeoff(odom_data);
                                ROS_INFO("\033[32m[FSM] MANUAL --> TAKEOFF(L1)\033[32m");
                            }
                        }
                    }else if(state_data.current_state.armed){ // already in flight
                        current_state = HOVER;
                        set_hov_with_odom();
                        ROS_INFO("\033[32m[FSM] MANUAL(L1) --> HOVER(L2)\033[32m");
                    }
                }
                break;
            
            case TAKEOFF:
                if(state_data.current_state.mode != "OFFBOARD"){
                    current_state = MANUAL;
                    ROS_INFO("\033[32m[FSM]:Exit OFFBOARD Mode, TAKEOFF --> MANUAL(L2)\033[32m");
                }else if(((now_time - takeoff_state.toggle_takeoff_time).toSec() < AutoTakeoff_t::MOTORS_SPEEDUP_TIME 
                            || (odom_data.p.head<2>() - param.takeoff_state.takeoff_pos.head<2>()).norm() > 0.8) && takeoff_state.first_time){
                    des = get_pv_speed_up_des(now_time);
                }else if(odom_data.p(2) >= (takeoff_state.start_pose(2) + param.takeoff_state.height-0.1)){ // reach desired height
                    set_hov_with_odom();
                    takeoff_state.delay_trigger.first = true;
                    takeoff_state.delay_trigger.second = now_time + ros::Duration(AutoTakeoff_t::DELAY_TRIGGER_TIME);

                    current_state = HOVER;

                    ROS_INFO("\033[32m[FSM] TAKEOFF --> HOVER(L2)\033[32m");
                }else{
                    if(takeoff_state.first_time){
                        set_start_pose_for_takeoff(odom_data);
                        takeoff_state.first_time = false;
                    }
                    des = get_takeoff_des(odom_data);
                }
                break;

            case HOVER:
                if(state_data.current_state.mode != "OFFBOARD"){
                    current_state = MANUAL;
                    ROS_INFO("\033[32m[FSM]:Exit OFFBOARD Mode, HOVER --> MANUAL(L2)\033[32m");
                }else if(cmd_is_received(now_time) && state_data.current_state.armed){
                    current_state = MISSION;
                    des = get_cmd_des();
                    ROS_INFO("\033[32m[FSM] HOVER --> MISSION(L2)\033[32m");
                }else if(!state_data.current_state.armed){
                    current_state = MANUAL;
                    ROS_INFO("\033[32m[FSM] HOVER --> MANUAL(L2)\033[32m");
                }else{
                    des = get_hover_des();

                    if(takeoff_state.delay_trigger.first && now_time > takeoff_state.delay_trigger.second){
                        takeoff_state.delay_trigger.first = false;
                        publish_trigger(odom_data.msg);
                        ROS_INFO("\033[32m[FSM]:TRIGGER sent, allow user command.\033[32m");
                    }  
                }
                break;

            case MISSION:
                if(state_data.current_state.mode != "OFFBOARD"){
                    current_state = MANUAL;
                    ROS_INFO("\033[32m[FSM]:Exit OFFBOARD Mode, MISSION --> MANUAL(L2)\033[32m");
                }else if (!cmd_is_received(now_time)){
                    current_state = HOVER;
                    set_hov_with_odom();
                    des = get_hover_des();
                    ROS_INFO("[FSM]:From MISSION(L3) to HOVER(L2)!");
                }
                else{
                    des = get_cmd_des();
                }
                break;
        }

        switch (param.controller_type)
        {
        case 0:
            //TODO position controller
            debug_msg = pos_controller.calculateControl(des, odom_data, u);
            publish_position_ctrl(u, now_time);
            break;

        case 1:
            debug_msg = vel_controller.calculateControl(des, odom_data, u);
            publish_velocity_ctrl(u, now_time);
            break;
        
        default:
            break;
        }

        debug_msg.header.stamp = now_time;
		debug_pub.publish(debug_msg);

        takeoff_triger_received = false;
    }
    
    void FSM::init(ros::NodeHandle& nh){
        param.config_from_ros_handle(nh);
        vel_controller.init(param);
        pos_controller.init(param);

        takeoff_triger_sub_ = nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land", 1, &FSM::takeoff_triger_callback, this, 
                                                                        ros::TransportHints().tcpNoDelay());

        state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, boost::bind(&State_Data_t::feed, &state_data, _1));

        extended_state_sub = nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state", 10,
                                                                        boost::bind(&ExtendedState_Data_t::feed, &extended_state_data, _1));

        odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10,
                                                    boost::bind(&Odom_Data_t::feed, &odom_data, _1),
                                                    ros::VoidConstPtr(),
                                                    ros::TransportHints().tcpNoDelay());

        cmd_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("cmd", 100,
                                                                boost::bind(&Command_Data_t::feed, &cmd_data, _1),
                                                                ros::VoidConstPtr(),
                                                                ros::TransportHints().tcpNoDelay());

        ctrl_pv_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
        ctrl_aw_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
        traj_start_triger_pub = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);
        debug_pub = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

        arming_client_srv = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

        int trials = 0;
        //判断是否连接上PX4
        while (!state_data.current_state.connected){
            ros::Duration(1.0).sleep();
            if (trials++ > 5)
                ROS_ERROR("Unable to connnect to PX4!!!");
        }

        current_state = MANUAL;
        fsm_timer_ = nh.createTimer(ros::Duration(1.0 / param.fsmparam.frequncy), &FSM::fsm_timer, this);

        ROS_INFO("\033[32m[FSM]:Init completed, change to MANUAL state!\033[32m");
    }
    
    void FSM::onInit(void){
        ros::NodeHandle nh(getMTPrivateNodeHandle());
        initThread_ = std::thread(std::bind(&FSM::init, this, nh));
    }

    bool FSM::odom_is_received(const ros::Time &now_time){
        return (now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom;
    }

    bool FSM::cmd_is_received(const ros::Time &now_time){
        return (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ctrl_node::FSM, nodelet::Nodelet);