#pragma once
#include "common_headers.h"
using namespace std;
using namespace Eigen;

struct ImuData{
    double _stamp;
    float _gyr_x, _gyr_y, _gyr_z;
    float _acc_x, _acc_y, _acc_z;
};

struct ImgData{
    double _stamp;
    string _filename;
};

class EurocDataset{
public:
    EurocDataset(const string cam_path, const string imu_path):_cam_path(cam_path), _imu_path(imu_path){
       _fp_img = fopen((_cam_path + "/data.csv").c_str(), "r");
       _fp_imu = fopen((_imu_path + "/data.csv").c_str(), "r");
    };
    ~EurocDataset(){
        // 指针置空防止野指针
        _fp_img = nullptr;
        _fp_imu = nullptr;
    }
    /**
     * @brief 读取Img图像文件编号以及IMU量测信息 
     * 
     * @return true 
     * @return false 
     */
    bool loadDataset();
    /**
     * @brief 发布imu消息 
     * 
     */
    void pubImuMsg();
    /**
     * @brief 发布img消息 
     * 
     */
    void pubImgMsg();
    //文件路径
    FILE *_fp_img, *_fp_imu;
    string _cam_path, _imu_path;
    //IMG数组,[time(ns),filename]
    queue<ImgData> _img_data;
    //IMU数组，[time(ns)，gyr_x(rad/s), gyr_y(rad/s), gyr_z(rad/s), acc_x(m/s^-2), acc_y(m/s^-2), acc_z(m/s^-2)]
    queue<ImuData> _imu_data;
    // 消息发布器
    ros::Publisher _pub_imu;
    image_transport::Publisher _pub_img;
    // 消息计数器
    int _img_seq = 0;
    int _imu_seq = 0;
};