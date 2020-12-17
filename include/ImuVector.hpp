#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include<vector>
#include<math.h>

#include "my_test_package/XYZRPY.h"

//结构体：构造带时间戳的四元数

struct QuaternionStamped
{
    ros::Time time;
    Eigen::Quaterniond q;

    QuaternionStamped(){

    };

    QuaternionStamped(const ros::Time& t,const Eigen::Quaterniond& q){
        this->time = t;

        Eigen::Quaterniond  q_tmp(q.w(),
                                  q.x(),
                                  q.y(),
                                  q.z());

        this->q = q_tmp;    
    };

    QuaternionStamped(const sensor_msgs::Imu& msg){
        
        this->time = msg.header.stamp;
        Eigen::Quaterniond  q_tmp(msg.orientation.w,
                                  msg.orientation.x,
                                  msg.orientation.y,
                                  msg.orientation.z);

        this->q = q_tmp;
    };
};

//将自己定义的带时间戳的四元数 转换为自己做的消息格式xyzrpy
void QuaternionStamped2XYZRPY(const QuaternionStamped& msgIn,my_test_package::XYZRPY& msgOut){
    
    geometry_msgs::Quaternion geo_q;

    geo_q.x = msgIn.q.x();
    geo_q.y = msgIn.q.y();
    geo_q.z = msgIn.q.z();
    geo_q.w = msgIn.q.w();
    
    tf::Quaternion q;
    tf::quaternionMsgToTF(geo_q,q);
    tf::Matrix3x3(q).getRPY(msgOut.roll,msgOut.pitch,msgOut.yaw); 

    msgOut.header.frame_id = "map";
    msgOut.header.stamp = msgIn.time;
}

my_test_package::XYZRPY Quaternion2XYZRPY(const Eigen::Quaterniond& msgIn){
    
    geometry_msgs::Quaternion geo_q;
    my_test_package::XYZRPY msgOut;

    geo_q.x = msgIn.x();
    geo_q.y = msgIn.y();
    geo_q.z = msgIn.z();
    geo_q.w = msgIn.w();
    
    tf::Quaternion q;
    tf::quaternionMsgToTF(geo_q,q);
    tf::Matrix3x3(q).getRPY(msgOut.roll,msgOut.pitch,msgOut.yaw); 

    msgOut.header.frame_id = "map";
    msgOut.header.stamp = ros::Time::now();

    return msgOut;
}

//四元数球面线性插值 result_q是靠近q2范围t，靠近q1（1-t）
Eigen::Quaterniond slerp(const Eigen::Quaterniond& q1,const Eigen::Quaterniond& q2,const float& t){
    
    float cosa = q1.x()*q2.x() + q1.y()*q2.y() + q1.z()*q2.z() +q1.w()*q2.w();

    // If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
    // the shorter path. Fix by reversing one quaternion.
    Eigen::Quaterniond tmp_q2;
    if(cosa< 0.0f){
        tmp_q2 = Eigen::Quaterniond(-q2.w(),-q2.x(),-q1.y(),-q1.z());
        cosa = -cosa;
    }

    float k0,k1;

    // If the inputs are too close for comfort, linearly interpolate
    if ( cosa > 0.9995f ) 
    {
        k0 = 1.0f - t;
        k1 = t;
    }else 
    {
        float sina = sqrt( 1.0f - cosa*cosa );
        float a = atan2( sina, cosa );
        k0 = sin((1.0f - t)*a)  / sina;
        k1 = sin(t*a) / sina;
    }

    return Eigen::Quaterniond(q1.w()*k0 +q2.w()*k1,
                              q1.x()*k0 +q2.x()*k1,
                              q1.y()*k0 +q2.y()*k1,
                              q1.z()*k0 +q2.z()*k1);

}

Eigen::Quaterniond calculateImuQuaternionDif(const Eigen::Quaterniond& curQ,
                                             const Eigen::Quaterniond& lastQ){

    Eigen::Matrix3d rotation;

    rotation = lastQ.toRotationMatrix().inverse()*curQ.toRotationMatrix();

    return Eigen::Quaterniond(rotation);
}


class ImuVectorHandler{
    
    public:
        
        //构造一个函数出来
        ImuVectorHandler():
        size(200),
        imuPointerLast(0),
        imuPointerFront(0),
        imuPointerLastIteration(0)
        {
            QuaternionStampedVector.clear();
            QuaternionStampedVector.resize(size);
        };
        
        //删去函数
        ~ImuVectorHandler(){};

        void push_back(const sensor_msgs::Imu& msg){

            imuPointerLast = (imuPointerLast + 1) % size;

            QuaternionStamped tmp(msg);

            QuaternionStampedVector[imuPointerLast] = tmp;

        };

        //找到FrontIMU和BackIMU 在时间戳顺序上： BackIMUTime lidarTime FrontIMU
        bool extractIMUData(const ros::Time& t,QuaternionStamped& data){

            double timeSeq = t.toSec();

            if(imuPointerLast >= 0){
                while(imuPointerFront != imuPointerLast){
                    
                    if (timeSeq < QuaternionStampedVector[imuPointerFront].time.toSec()) {
                        break;
                    }

                    imuPointerFront = (imuPointerFront + 1) % size;
                }

                if (timeSeq > QuaternionStampedVector[imuPointerFront].time.toSec()) {

                    std::cout<<"time dif: "<<timeSeq - QuaternionStampedVector[imuPointerFront].time.toSec()<<std::endl;
                    data = QuaternionStampedVector[imuPointerFront];
                    ROS_ERROR("extractIMUData error");
                    return false;
                }else{

                    int imuPointerBack = (imuPointerFront + size - 1) % size;

                    data = ImuTimeInterpolation(t,imuPointerBack);

                    std::cout<<"time dif: "<<timeSeq - QuaternionStampedVector[imuPointerFront].time.toSec()<<std::endl;

                }
                
            }

            imuPointerLastIteration = imuPointerLast;

            return true;
        };

        //将imu[frontier]和imu[imuPointerBack]进行线性时间插值，插到激光雷达时间上去
        QuaternionStamped ImuTimeInterpolation(const ros::Time& t,const int& imuPointerBack){

            double timeFront = QuaternionStampedVector[imuPointerFront].time.toSec();
            double timeBack = QuaternionStampedVector[imuPointerBack].time.toSec();
            double tSeq = t.toSec();

            double ratioFront = (timeFront - tSeq)/(timeFront - timeBack);
            double ratioBack = (tSeq - timeBack)/(timeFront - timeBack);
            
            float ratio = ratioFront/(ratioFront + ratioBack);

            Eigen::Quaterniond q = slerp(QuaternionStampedVector[imuPointerBack].q,
                                        QuaternionStampedVector[imuPointerFront].q,
                                        ratio);

            QuaternionStamped qt(t,q);

            return qt;

        };

    private:

        
        std::vector<QuaternionStamped> QuaternionStampedVector;

        int size;

        int imuPointerLast;

        int imuPointerFront;

        int imuPointerLastIteration;


};

