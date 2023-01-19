# My-SLAM-Training

## Introduction

这是我在学习完《视觉SLAM十四讲》和ORBSLAM2之后，想要对所学知识进行的一个总结：简单手写一个VO系统

目前还是有很多问题的，但是由于后续的工作学习需要，这里暂时告一段落。

![微信图片_20230118194142](https://gitee.com/zhangzuoos/pic-go/raw/master/picgo/微信图片_20230118194142.png)

* 整体的框架基本上是跟ORBSLAM2的差不多，除了没有重定位和回环检测，使用ROS平台，简写了一个简单的VO系统
* 使用了ROS的服务通讯框架，feature_tracking和local_map组成了客户端和服务端，分别对应ORBSLAM2中的Tracking和LocalMapping两个主要线程（实际上应使用节点通讯，但是节点通讯的代码出现了问题，目前没有上传），使用自建的srv消息数据结构，作为两端之间的通信数据
* 整个代码使用了OpenCV、G2O和Sophus等库
* 运行使用的是Kitti数据集

## Problems

虽然现在能够完整的运行完Kitti-07数据集，但是从上面的图中就能看出来一下问题：

* 由于目前的Demo没有添加回环检测，因此能够看到累计误差越来越大
* 由于使用的是服务通讯，是请求相应的方式进行通讯的，因此运行速度非常慢
* 每帧之间匹配的地图点数量不如ORBSLAM2的多（主要是因为ORBSLAM2中使用了均匀提取角点，而我直接使用了OpenCV中的函数对整个图片进行提取特征点，因此质量不如ORBSLAM2的好）

## Prerequisites

* Ubuntu 和 ROS Ubuntu 18.04 melodic 
* 使用的各类库，基本上与ORBSLAM2相同

## Run on ROS

数据集：使用了kitti2rosbag工具，将下载的kitti图片转换为bag文件

词袋文件：将ORBSLAM2的词袋文件存放在config -> ORBvoc.txt

建立ROS工作空间：

    cd ~/catkin_ws/src
    git clone https://github.com/zhang-zuo/my-slam-training.git
    cd ../
    catkin_make
    source ~/catkin_ws/devel/setup.bash

运行roslaunch：**

    roslaunch feature_tracking my_slam.launch
    rosrun rviz rviz
    rosbag play YOUR_PATH_TO_DATASET/kitti2rosbag.bag 

打开空的RVIZ后，加载git仓库中的Config文件：file -> Open Config -> my-slam.rviz

**可能遇到的问题：**

* 问题：使用cv_bridge转换OpenCV类型图片时，节点会挂掉
* * 解决：cv_bridge使用的是ROS自带的OpenCV（我的Melodic版本ROS自带OpenCV-3.2.0）,而正常使用OpenCV是使用的自己安装的3.4.6版本（ORBSLAM2使用的），两个版本使用出现冲突，因此需要关掉一个使用另一个。可百度找到方法，将cv_bridge调用OpenCV时使用3.4.6版本。