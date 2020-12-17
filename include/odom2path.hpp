#pragma once

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "my_test_package/XYZRPY.h"
#include "ImuVector.hpp"

double odomSD(const nav_msgs::Odometry& last_msg,const nav_msgs::Odometry& now_msg){
    
    double x1 = last_msg.pose.pose.position.x;
    double y1 = last_msg.pose.pose.position.y;
    double z1 = last_msg.pose.pose.position.z;

    double x2 = now_msg.pose.pose.position.x;
    double y2 = now_msg.pose.pose.position.y;
    double z2 = now_msg.pose.pose.position.z;

    double sq = std::sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)+(z1-z2)*(z1-z2));

    return sq;
}



double yawdif(const nav_msgs::Odometry& last_msg,const nav_msgs::Odometry& now_msg){

    double lastYaw = tf::getYaw(last_msg.pose.pose.orientation);
    double nowYaw = tf::getYaw(now_msg.pose.pose.orientation);

    double dYaw = nowYaw - lastYaw;

    if(dYaw > 2*M_PI){
        dYaw-=2*M_PI;
    }else if(dYaw < -2*M_PI){
        dYaw+=2*M_PI;
    }
    
    double dYawSQ = std::sqrt(dYaw*dYaw);

    return dYawSQ;
}

my_test_package::XYZRPY odom2XYZRPY(const nav_msgs::Odometry& msg){

    my_test_package::XYZRPY tmp;

    tmp.header = msg.header;
    tmp.x = msg.pose.pose.position.x;
    tmp.y = msg.pose.pose.position.y;
    tmp.z = msg.pose.pose.position.z;
    
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.pose.orientation,q);
    tf::Matrix3x3(q).getRPY(tmp.roll,tmp.pitch,tmp.yaw); 

    return tmp;

}

my_test_package::XYZRPY imu2XYZRPY(const sensor_msgs::Imu & msg){

    my_test_package::XYZRPY tmp;

    tmp.header = msg.header;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.orientation,q);
    tf::Matrix3x3(q).getRPY(tmp.roll,tmp.pitch,tmp.yaw); 

    return tmp;

}

void publishMy6D(const nav_msgs::Odometry& msg,ros::Publisher pub){
    
    my_test_package::XYZRPY tmp = odom2XYZRPY(msg);
    pub.publish(tmp);

}

void publishMy6D(const Eigen::Quaterniond& msg,ros::Publisher pub){
    
    my_test_package::XYZRPY tmp = Quaternion2XYZRPY(msg);
    pub.publish(tmp);

}

void legoOdom2Map(const nav_msgs::Odometry& msgIn,nav_msgs::Odometry& msgOut){

    msgOut = msgIn;

    msgOut.header.frame_id = "map";

    Eigen::Matrix4d TworldlegoZXY;

    double x = msgIn.pose.pose.position.x;
    double y = msgIn.pose.pose.position.y;
    double z = msgIn.pose.pose.position.z;

    Eigen::Vector3d t1(x,y,z),t2;

    Eigen::Quaterniond q(msgIn.pose.pose.orientation.w,
                        msgIn.pose.pose.orientation.x,
                        msgIn.pose.pose.orientation.y,
                        msgIn.pose.pose.orientation.z);

    Eigen::Matrix3d rotation = q.toRotationMatrix();

    Eigen::Matrix3d rotation2;

    //这里为什么是M_PI_2 ，其实不符合逻辑推导的过程。
    Eigen::Vector3d ea(M_PI_2, 0, M_PI_2);//先旋转x，再y，再z

    Eigen::Matrix3d rotation_matrix3;
    rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());

    t2 = rotation_matrix3 * t1;

    rotation2 = rotation_matrix3 * rotation;

    Eigen::Quaterniond q2(rotation2);

    msgOut.pose.pose.position.x = t2(0) +10;
    msgOut.pose.pose.position.y = t2(1) +10;
    msgOut.pose.pose.position.z = t2(2);

    msgOut.pose.pose.orientation.x = q2.x();
    msgOut.pose.pose.orientation.y = q2.y();
    msgOut.pose.pose.orientation.z = q2.z();
    msgOut.pose.pose.orientation.w = q2.w();

}