# 郭子萱的测试程序包
---

目前集成有以下功能：

1、将gazebo的odom变成path发布在rviz中。
2、将lego_loam的laser_odom_to_init变成path发布在rviz中。
3、自定义消息类型：XYZRPY 将odom和imu信息转为XYZRPY进行发布。
4、同步时间戳，获得imu中值数据。
5、


---

待做功能：

1、计算当前时刻计算出来的imu数据和上一时刻的imu姿态角差值。