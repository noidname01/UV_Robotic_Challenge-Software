#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"hello_cpp_node");
    
    ros::NodeHandle handler;
    while(ros::ok()){
        ROS_INFO("Hello World~");
        ros::Duration(1).sleep();
    }
    
}