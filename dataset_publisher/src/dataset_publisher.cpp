#include "common_headers.h"
#include "dataset_publisher/hitll.h"
using namespace std;

int main(int argc, char** argv){
    ros::init(argc, argv, "dataset_publisher");
    ros::NodeHandle nh("~");
    
    HITLL::Ptr hitll_ptr = HITLL::createHITLL("/home/yuntian/lunarDataset/Lander300/Handhold");
    hitll_ptr->loadDataset();
    // ================ 注册发布器 ================== //
    image_transport::ImageTransport it(nh);
    hitll_ptr->_mynt_img_pub = it.advertise("/mynteye/cam0/image_raw", 100);
    hitll_ptr->_mynt_imu_pub = nh.advertise<sensor_msgs::Imu>("/mynteye/imu0/data_raw", 1000);
    hitll_ptr->_pix_imu_pub = nh.advertise<sensor_msgs::Imu>("/pixhawk/imu0/data_raw", 1000);
    hitll_ptr->_pix_path_pub = nh.advertise<nav_msgs::Path>("/pixhawk/local_path/path", 500);
    hitll_ptr->_opt_path_pub = nh.advertise<nav_msgs::Path>("/optitrack/local_path/path", 500);
    // ================ 发布数据集 ================== //
    ros::Rate loop_rate(200);
    size_t total_imu = hitll_ptr->_mynt_imu_data.size();
    size_t count = 0;
    while(ros::ok() && count < total_imu){
        count++;
        ROS_INFO("IMU count: %lu.", count);
        // 发布Mynteye图像
        if(hitll_ptr->_mynt_imu_data.front()._stamp >= hitll_ptr->_mynt_img_data.front()._stamp
            && hitll_ptr->_mynt_img_data.size() != 0)
        {
            hitll_ptr->pubMyntImg();
        }
        // 发布ground_truth
        if(hitll_ptr->_mynt_imu_data.front()._stamp >= hitll_ptr->_opt_pose_data.front()._stamp
            && hitll_ptr->_opt_pose_data.size() != 0)
        {
            hitll_ptr->pubOptPath();
        }

        hitll_ptr->pubMyntImu();
        loop_rate.sleep();
    }

    return 0;
}