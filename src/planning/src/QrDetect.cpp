#include "QrDetect.hpp"


int main(int argc, char *argv[]){
    ros::init(argc, argv, "QrDetect");
    ros::NodeHandle nh("~");
    QrDetect qrDetect(nh);

    ros::Timer timer = nh.createTimer(ros::Duration(0.1), 
        [&qrDetect](const ros::TimerEvent&) { 
            qrDetect.mainLoop(); 
        });

    ros::spin();
}