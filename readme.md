# 郭子萱的测试程序包
---

目前集成有以下功能：

1、将gazebo的odom变成path发布在rviz中。
2、将lego_loam的/laser_odom_to_init变成path发布在rviz中。
3、将lego_loam的/aft_mapped_to_init变成path发布在rviz中。
3、自定义消息类型：XYZRPY 将odom和imu信息转为XYZRPY进行发布。


注：lego2map中为了在班超的gazebo仿真环境中测试，加了一些数据转换的东西

---
rviz中 红色为/laser_odom_to_init保存的path
rviz中 蓝色为/aft_mapped_to_init保存的path
rviz中 绿色为odom保存的path