#include "QrDetect.hpp"


int main(int argc, char *argv[]){
    ros::init(argc, argv, "QrDetect");
    ros::NodeHandle nh("~");
    QrDetect qrDetect(nh);
    ros::spin();
}