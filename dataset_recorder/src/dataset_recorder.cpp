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
using namespace std;
// =================================== 全局变量 =========================== //
FILE *fp_mynt_imu, *fp_mynt_img;
FILE *fp_pix_imu, *fp_pix_pose;
FILE *fp_opt_pose;

int mynt_imu_count, mynt_img_count;
int pix_imu_count, pix_pose_count;
int opt_imu_count;
// ================================= 回调记录函数 ========================== //
void myntImgCallback(const sensor_msgs::ImageConstPtr &img_msg){

}

void myntImuCallback(const sensor_msgs::ImuConstPtr &imu_msg){

}

void pixImuCallback(const sensor_msgs::ImuConstPtr &imu_msg){

}

void pixPoseCallback(const geometry_msgs::PoseStamped &pose_msg){

}

void optPoseCallback(const geometry_msgs::PoseStamped &pose_msg){

}

bool openFiles(const string& dataset_path);
// ================================== 节点主函数 ============================ //
int main(int argc, char** argv){
    ros::init(argc, argv, "dataset_recorder");
    ros::NodeHandle nh("~");

    if(argc!=2){
        ROS_WARN("Not enough arguments!");
        return -1;
    }

    string dataset_path = argv[2];
    // ==================== 清空并创建数据集目录 ===================== //
    system(("rm -rf " + dataset_path).c_str());
    system(("mkdir -p " + dataset_path + "/mynteye/cam0/data").c_str());    
    system(("mkdir -p " + dataset_path + "/mynteye/imu0").c_str());    
    system(("mkdir -p " + dataset_path + "/ground_truth").c_str());    
    system(("mkdir -p " + dataset_path + "/pixhawk").c_str());    
    // ======================= 写入文件头 =========================== //


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
    fp_mynt_imu = fopen((dataset_path + "/mynteye/imu0/data.csv").c_str(), "w+");
    if(fp_mynt_imu == nullptr){
        ROS_WARN("Failed to write mynteye imu datafile!");
        return false;
    }
    else{
        fprintf(fp_mynt_imu, "time_stamp(ns),acc_x(g),acc_y(g),acc_z(g),gyr_x(deg/s),gyr_y(deg/s),gyr_z(deg/s)\n");
    }
    // 打开并写入Optitrack Pose数据文件头


}
