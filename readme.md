# 郭子萱的测试程序包
---

目前集成有以下功能：

* 将gazebo的odom变成path发布在rviz中。
* 将lego_loam的/laser_odom_to_init变成path发布在rviz中。
* 将lego_loam的/aft_mapped_to_init变成path发布在rviz中。
* 自定义消息类型：XYZRPY 将odom和imu信息转为XYZRPY进行发布。


注：lego2map中为了在班超的gazebo仿真环境中测试，加了一些数据转换的东西

>rviz中 红色为/laser_odom_to_init保存的path
>rviz中 蓝色为/aft_mapped_to_init保存的path
>rviz中 绿色为odom保存的path


### 新增timeContrast节点

请将topic名称按照需求在timeContrast.launch进行修改

仿真环境:
```bash
roscore
`open your publish message node `
roslaunch my_test_package timeContrast.launch
```

注:如果在现实使用,注意修改是否使用仿真时间

### 新增导出txt功能(odom2path.cpp)

python中使用的是绝对路径

数据包播放完成后，可以通过以下指令来进行txt文件导出

```bash
rostopic pub /txt_flag std_msgs/Empty "{}"
```

### 绘图功能（draw.py）

须给draw.py增加可执行权限

```bash
chmod +x draw.py
```

我绘制出的图像在`image/折线图/`中