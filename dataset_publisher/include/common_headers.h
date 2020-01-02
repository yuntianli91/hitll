/**
 * @file common_headers.h
 * @author yuntian li (yuntianlee91@hotmail.com)
 * @brief 
 * @version 0.1
 * @date 2020-01-02
 * 
 * @copyright Copyright (c) 2020
 * 
 */
#ifndef _COMMON_HEADER_H
#define _COMMON_HEADER_H
// ================== ROS相关头文件 =================== //
#include <ros/ros.h> //核心库
#include <std_msgs/Header.h> //消息头
#include <std_msgs/Bool.h> //布尔消息
#include <sensor_msgs/Imu.h> //IMU消息
#include <sensor_msgs/Image.h> //图像消息
#include <sensor_msgs/image_encodings.h> //图像消息编码
#include <sensor_msgs/Range.h> //测距消息
#include <sensor_msgs/PointCloud.h> //传统点云数据
#include <sensor_msgs/PointCloud2.h> //新版本可和PCL交互的点云数据
#include <image_transport/image_transport.h> // 
#include <geometry_msgs/Pose.h> //位姿消息
#include <cv_bridge/cv_bridge.h> //ROS与OpenCV转换
#include <nav_msgs/Odometry.h> //里程计消息
#include <nav_msgs/Path.h> //轨迹消息
#include <tf2_eigen/tf2_eigen.h> //tf2与eigen函数
#include <tf2_ros/transform_listener.h> //坐标变换接收
#include <tf2_ros/transform_broadcaster.h> //坐标变换发布
#include <tf2/LinearMath/Quaternion.h> //四元数
#include <geometry_msgs/TransformStamped.h> //带时戳的坐标变换消息
// ================== C++标准库头文件 ================= //
#include <stdlib.h> //标准库
#include <memory> //智能指针库
#include <cmath> //数学运算库
#include <mutex> //智能锁库(多线程)
#include <thread> //线程库(多线程)
#include <condition_variable> //条件变量库（多线程）
#include <cstdio> //C标准库
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <string> // 字符串
#include <map> //map容器
#include <vector> // vector容器
#include <queue> // queue容器
// ================== 第三方库头文件 ================== //
#include <Eigen/Dense>
#include <opencv2/opencv.hpp> //OpenCV
#include <opencv2/core/eigen.hpp> //OpenCV与Eigen转换
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#endif