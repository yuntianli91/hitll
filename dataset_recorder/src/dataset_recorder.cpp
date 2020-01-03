#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
using namespace std;
// =================================== 全局变量 =========================== //
FILE *fp_mynt_imu, *fp_mynt_img;
FILE *fp_pix_imu, *fp_pix_pose;
FILE *fp_opt_pose;

int mynt_imu_count, mynt_img_count;
int pix_imu_count, pix_pose_count;
int opt_imu_count;

string dataset_path;
// ================================= 回调记录函数 ========================== //
void myntImgCallback(const sensor_msgs::ImageConstPtr &img_msg){
    // 记录图像时序文件
    string img_filename = dataset_path + "/mynteye/cam0/data/" + to_string(img_msg->header.stamp.toNSec()) + ".png";
    fprintf(fp_mynt_img, "%lu,%s", 
            img_msg->header.stamp.toNSec(),
            img_filename.c_str());
    // 记录图像数据
    cv::imwrite(img_filename, cv_bridge::toCvShare(img_msg, "mono8")->image);
}

void myntImuCallback(const sensor_msgs::ImuConstPtr &imu_msg){
    fprintf(fp_mynt_imu, "%lu,%f,%f,%f,%f,%f,%f",
            imu_msg->header.stamp.toNSec(),
            imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
            imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
}

void pixImuCallback(const sensor_msgs::ImuConstPtr &imu_msg){
    fprintf(fp_pix_imu, "%lu,%f,%f,%f,%f,%f,%f",
            imu_msg->header.stamp.toNSec(),
            imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
            imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
}

void pixPoseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg){
    fprintf(fp_pix_pose, "%lu,%f,%f,%f,%f,%f,%f,%f",
            pose_msg->header.stamp.toNSec(),
            pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z, 
            pose_msg->pose.orientation.w, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
}

void optPoseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg){
    fprintf(fp_opt_pose, "%lu,%f,%f,%f,%f,%f,%f,%f",
            pose_msg->header.stamp.toNSec(),
            pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z, 
            pose_msg->pose.orientation.w, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
}

bool openFiles(const string& dataset_path);
void closeFiles();
// ================================== 节点主函数 ============================ //
int main(int argc, char** argv){
    ros::init(argc, argv, "dataset_recorder");
    ros::NodeHandle nh("~");

    string dataset_path = argv[2];
    nh.getParam("dataset_path",dataset_path);
    // ----------------------- 清空并创建数据集目录 ------------------------ //
    system(("rm -rf " + dataset_path).c_str());
    system(("mkdir -p " + dataset_path + "/mynteye/cam0/data").c_str());    
    system(("mkdir -p " + dataset_path + "/mynteye/imu0").c_str());    
    system(("mkdir -p " + dataset_path + "/ground_truth").c_str());    
    system(("mkdir -p " + dataset_path + "/pixhawk").c_str());    
    // -------------------------- 写入文件头 ----------------------------- //
    if(!openFiles(dataset_path)){
        return -1;
    }
    // -------------------------- 订阅量测消息 ---------------------------- //
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber mynt_img_sub = it.subscribe("/mynteye_ros_node/cam0/image_raw", 100, myntImgCallback);
    ros::Subscriber mynt_imu_sub = nh.subscribe("/mynteye_ros_node/imu0/data_raw", 1000, myntImuCallback);
    ros::Subscriber pix_imu_sub = nh.subscribe("/mavros/imu/data_raw", 500, pixImuCallback);
    ros::Subscriber pix_pose_sub = nh.subscribe("/mavros/local_pose/pose", 500, pixPoseCallback);
    ros::Subscriber opt_pose_sub = nh.subscribe("/vrpn_ros_node/Lander300/pose", 500, optPoseCallback);
    int count = 0;
    while(ros::ok()){
        ros::spinOnce();
        ROS_INFO("count: %u", count);
    }

    closeFiles();
    return 0;
}

bool openFiles(const string& dataset_path){
    // 打开并写入Mynteye Img数据文件头
    fp_mynt_img = fopen((dataset_path + "/mynteye/cam0/data.csv").c_str(), "w+");
    if(fp_mynt_img == nullptr){
        ROS_WARN("Failed to write mynteye img datafile!");
        return false;
    }
    else{
        fprintf(fp_mynt_img, "time_stamp(ns),filename\n");
    }
    // 打开并写入Mynteye Imu数据文件头
    fp_mynt_imu = fopen((dataset_path + "/mynteye/imu0/data.csv").c_str(), "w+");
    if(fp_mynt_imu == nullptr){
        ROS_WARN("Failed to write mynteye imu datafile!");
        return false;
    }
    else{
        fprintf(fp_mynt_imu, "time_stamp(ns),acc_x(g),acc_y(g),acc_z(g),gyr_x(deg/s),gyr_y(deg/s),gyr_z(deg/s)\n");
    }
    // 打开并写入Pixhawk Imu数据文件头
    fp_pix_imu = fopen((dataset_path + "/pixhawk/imu.csv").c_str(), "w+");
    if(fp_pix_imu == nullptr){
        ROS_WARN("Failed to write pixhawk imu datafile!");
        return false;
    }
    else{
        fprintf(fp_pix_imu, "time_stamp(ns),acc_x(g),acc_y(g),acc_z(g),gyr_x(deg/s),gyr_y(deg/s),gyr_z(deg/s)\n");
    }
    // 打开并写入Pixhawk Pose数据文件头
    fp_pix_pose = fopen((dataset_path + "/pixhawk/pose.csv").c_str(), "w+");
    if(fp_mynt_imu == nullptr){
        ROS_WARN("Failed to write pixhawk pose datafile!");
        return false;
    }
    else{
        fprintf(fp_pix_pose, "time_stamp(ns),pos_x(m),pos_y(m),pos_z(m),quat_w,quat_x,quat_y,quat_z\n");
    }
    // 打开并写入Optitrack Pose数据文件头
    fp_opt_pose = fopen((dataset_path + "/ground_truth/data.csv").c_str(), "w+");
    if(fp_mynt_imu == nullptr){
        ROS_WARN("Failed to write optitrack pose datafile!");
        return false;
    }
    else{
        fprintf(fp_opt_pose, "time_stamp(ns),pos_x(m),pos_y(m),pos_z(m),quat_w,quat_x,quat_y,quat_z\n");
    }

    return true;
}

void closeFiles(){
    fclose(fp_mynt_imu); fclose(fp_mynt_img);
    fclose(fp_pix_imu); fclose(fp_pix_pose);
    fclose(fp_opt_pose);

    fp_mynt_img = nullptr; fp_mynt_imu = nullptr;
    fp_pix_pose = nullptr; fp_pix_imu = nullptr;
    fp_opt_pose = nullptr;
}
