#pragma once
#include "common_headers.h"
using namespace std;

struct ImuData{
    double _stamp;
    float _gyr_x, _gyr_y, _gyr_z;
    float _acc_x, _acc_y, _acc_z;
};

struct ImgData{
    double _stamp;
    string _filename;
};

struct PoseData
{
    double _stamp;
    float _pos_x, _pos_y, _pos_z;
    float _ori_w, _ori_x, _ori_y, _ori_z;
};

class HITLL{
public:
    typedef shared_ptr<HITLL> Ptr;
    // -------------------------- 函数 ---------------------- //
    HITLL(const string &data_path):_data_path(data_path){
        _fp_mynt_img = fopen((_data_path + "/mynteye/cam0/data.csv").c_str(), "r+");
        _fp_mynt_imu = fopen((_data_path + "/mynteye/imu0/data.csv").c_str(), "r+");
        _fp_pix_imu = fopen((_data_path + "/pixhawk/imu.csv").c_str(), "r+");
        _fp_pix_pose = fopen((_data_path + "/pixhawk/local_pose.csv").c_str(), "r+");
        _fp_opt_pose = fopen((_data_path + "/ground_truth/data.csv").c_str(), "r+");
    }
    ~HITLL(){
        fclose(_fp_mynt_img); fclose(_fp_mynt_imu);
        fclose(_fp_pix_imu); fclose(_fp_pix_pose);
        fclose(_fp_opt_pose);

        _fp_mynt_img = nullptr; _fp_mynt_imu = nullptr;
        _fp_pix_imu = nullptr; _fp_pix_pose = nullptr;
        _fp_opt_pose = nullptr;
    }
    void loadDataset();
    void pubMyntImg();
    void pubMyntImu();
    void pubPixImu();
    void pubPixPath();
    void pubOptPath();

    static Ptr createHITLL(const string &data_path){
        return(Ptr(new HITLL(data_path)));
    }
    // --------------------------变量 ------------------------ //
    // ros publisher
    ros::Publisher _mynt_imu_pub;
    image_transport::Publisher _mynt_img_pub;
    ros::Publisher _pix_imu_pub;
    ros::Publisher _pix_path_pub;
    ros::Publisher _opt_path_pub;
    // data container
    queue<ImgData> _mynt_img_data;
    queue<ImuData> _mynt_imu_data;
    queue<ImuData> _pix_imu_data;
    queue<PoseData> _pix_pose_data;
    queue<PoseData> _opt_pose_data;
private:
    string _data_path;

    FILE *_fp_mynt_img, *_fp_mynt_imu;
    FILE *_fp_pix_imu, *_fp_pix_pose;
    FILE *_fp_opt_pose;

    int _mynt_img_count = 0; 
    int _mynt_imu_count = 0; 
    int _pix_imu_count = 0; 
    int _pix_pose_count = 0; 
    int _opt_pose_count = 0; 

    nav_msgs::PathPtr _opt_path_ptr;
    nav_msgs::PathPtr _pix_path_ptr;
};
