#include <ros/ros.h>

#include <nav_msgs/Odometry.h>



void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg){

    double msgTime = msg->header.stamp.toSec();
    double nowTime = ros::Time::now().toSec();

    double timedif = nowTime - msgTime;

    std::cout<<"time difference: "<<timedif<<std::endl;

    if(timedif >= 0.5){
        ROS_ERROR("time difference is over 0.5s");
    }
}

int main(int argc,char **argv){

    ros::init(argc,argv,"time_contrast_node");

    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    ros::Subscriber odomSub;

    std::string odomTopic;

    if(!p_nh.getParam("odomTopic",odomTopic)){
        
        odomTopic = "/robot1";
    }

    //在这里改一下topic
    odomSub = nh.subscribe(odomTopic,1,odomCallBack);

    ros::spin();
}