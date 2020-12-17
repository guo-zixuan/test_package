#include "odom2path.hpp"


ros::Publisher odomPathPub,lidarOdomPub,odom6DPub,lidarOdom6DPub;
ros::Publisher lowOdomPathPub,lowOdom6DPub;
ros::Publisher imu6DPub,difImu6DPub;
nav_msgs::Path gazeboPath,lidarPath,lowLidarPath;

nav_msgs::Odometry lastGazeboOdom;
nav_msgs::Odometry lastLidarOdom;
nav_msgs::Odometry lastLowLidarOdom;

Eigen::Quaterniond lastImuQuaternion;

bool GazeboOdomFirstFlag = false;
bool LidarOdomFirstFlag = false;
bool LowLidarOdomFirstFlag = false;
bool ImuDataFirstFlag = false;

ImuVectorHandler imuVectorHandler;

bool extractImuQuaternion(ImuVectorHandler imuVectorHandler,
                          const ros::Time& lidarTime,
                          Eigen::Quaterniond& qOut){

    QuaternionStamped qt_tmp;

    if(imuVectorHandler.extractIMUData(lidarTime,qt_tmp)){
        std::cout<<"extractIMUData True"<<std::endl;

        Eigen::Quaterniond q(qt_tmp.q.w(),
                             qt_tmp.q.x(),
                             qt_tmp.q.y(),
                             qt_tmp.q.z() );

        qOut = q;

        return true;
    }

    return false;
}


void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg){

    if(GazeboOdomFirstFlag == false){

        gazeboPath.header.frame_id = "map";
        gazeboPath.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped tmpPose;
        tmpPose.header = msg->header;
        tmpPose.pose.position = msg->pose.pose.position;
        tmpPose.pose.orientation = msg->pose.pose.orientation;

        gazeboPath.poses.push_back(tmpPose);
        odomPathPub.publish(gazeboPath);

        GazeboOdomFirstFlag = true;
        lastGazeboOdom = *msg;
        
    }else if(GazeboOdomFirstFlag == true){

        if(odomSD(lastGazeboOdom,*msg)>= 0.05|| yawdif(lastGazeboOdom,*msg) >= 0.01){
            geometry_msgs::PoseStamped tmpPose;
            tmpPose.header = msg->header;
            tmpPose.pose.position = msg->pose.pose.position;
            tmpPose.pose.orientation = msg->pose.pose.orientation;

            gazeboPath.poses.push_back(tmpPose);
            odomPathPub.publish(gazeboPath);

            lastGazeboOdom = *msg;
        }
    }
    publishMy6D(*msg,odom6DPub);
}

void lowLidarOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg){

    if(LowLidarOdomFirstFlag == false){

        lowLidarPath.header.frame_id = "map";
        lowLidarPath.header.stamp = ros::Time::now();

        nav_msgs::Odometry tmpOdom;
        legoOdom2Map(*msg,tmpOdom);

        geometry_msgs::PoseStamped tmpPose;
        tmpPose.header = tmpOdom.header;
        tmpPose.pose.position = tmpOdom.pose.pose.position;
        tmpPose.pose.orientation = tmpOdom.pose.pose.orientation;

        lowLidarPath.poses.push_back(tmpPose);
        lowOdomPathPub.publish(lowLidarPath);

        publishMy6D(tmpOdom,lowOdom6DPub);

        LowLidarOdomFirstFlag = true;
        lastLowLidarOdom = *msg;
        
    }else if(LowLidarOdomFirstFlag == true){

        if(odomSD(lastLowLidarOdom,*msg)>= 0.05|| yawdif(lastLowLidarOdom,*msg) >= 0.01){

            nav_msgs::Odometry tmpOdom;
            legoOdom2Map(*msg,tmpOdom);    

            geometry_msgs::PoseStamped tmpPose;
            tmpPose.header = tmpOdom.header;
            tmpPose.pose.position = tmpOdom.pose.pose.position;
            tmpPose.pose.orientation = tmpOdom.pose.pose.orientation;

            lowLidarPath.poses.push_back(tmpPose);
            lowOdomPathPub.publish(lowLidarPath);

            publishMy6D(tmpOdom,lowOdom6DPub);

            lastLowLidarOdom = *msg;
        }
    }
    
}

//lidar odom 先从 x-y-z 转换到z-x-y试试
void lidarOdomCallBack(const nav_msgs::Odometry::ConstPtr& msg){

        ros::Time lidarTime = msg->header.stamp;

        if(LidarOdomFirstFlag == false){

            lidarPath.header.frame_id = "map";
            lidarPath.header.stamp = ros::Time::now();

            geometry_msgs::PoseStamped tmpPose;
            tmpPose.header.frame_id = "map";
            tmpPose.header.stamp = ros::Time::now();

            nav_msgs::Odometry tmpOdom;
            legoOdom2Map(*msg,tmpOdom);

            tmpPose.pose = tmpOdom.pose.pose;

            lidarPath.poses.push_back(tmpPose);
            lidarOdomPub.publish(lidarPath);


            LidarOdomFirstFlag = true;
            lastLidarOdom = *msg;
            
        }else if(LidarOdomFirstFlag == true){

            if(odomSD(lastLidarOdom,*msg)>= 0.05|| yawdif(lastLidarOdom,*msg) >= 0.01){
                geometry_msgs::PoseStamped tmpPose;
                tmpPose.header.frame_id = "map";
                tmpPose.header.stamp = ros::Time::now();

                nav_msgs::Odometry tmpOdom;
                legoOdom2Map(*msg,tmpOdom);
                
                publishMy6D(tmpOdom,lidarOdom6DPub);

                tmpPose.pose = tmpOdom.pose.pose;

                lidarPath.poses.push_back(tmpPose);
                lidarOdomPub.publish(lidarPath);

                lastLidarOdom = *msg;
            }
        }
}

void imuCallBack(const sensor_msgs::Imu::ConstPtr & msg){

    imuVectorHandler.push_back(*msg);
    ImuDataFirstFlag = true;
    
}

int main(int argc,char **argv){

    ros::init(argc,argv,"path_contrast_node");
    ros::NodeHandle nh;

    ros::Subscriber odomSub,lidarOdomSub,lowLidarOdomSub,imuSub;

    odomSub = nh.subscribe("/robot1/odom",1,odomCallBack);
    lidarOdomSub = nh.subscribe("/laser_odom_to_init",1,lidarOdomCallBack);
    imuSub = nh.subscribe("/imu",1,imuCallBack);
    lowLidarOdomSub = nh.subscribe("/aft_mapped_to_init",1,lowLidarOdomCallBack);

    odomPathPub = nh.advertise<nav_msgs::Path>("/gazebo_odom_path",1);
    lidarOdomPub = nh.advertise<nav_msgs::Path>("/high_lidar_odom_path",1);
    lowOdomPathPub = nh.advertise<nav_msgs::Path>("/low_lidar_odom_path",1);

    odom6DPub = nh.advertise<my_test_package::XYZRPY>("/gazebo_6D",1);
    lidarOdom6DPub = nh.advertise<my_test_package::XYZRPY>("/high_lidar_odom_6D",1);
    lowOdom6DPub = nh.advertise<my_test_package::XYZRPY>("/low_lidar_odom_6D",1);
    imu6DPub = nh.advertise<my_test_package::XYZRPY>("/imu_6D",1);
    difImu6DPub = nh.advertise<my_test_package::XYZRPY>("/dif_imu_6D",1);

    ros::spin();
}