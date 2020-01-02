#include "euroc.h"

bool EurocDataset::loadDataset(){
    if(_fp_img == NULL){
        cerr << "Failed to open imu data.csv!\n"; 
        return false;
    }
    if(_fp_imu == NULL){
        cerr << "Failed to open img data.csv!\n"; 
        return false;
    }
    int count = 0;
    //-------------------- 读取图像数据 ----------------- //
    char fileheader[200];
    if(fgets(fileheader, 200, _fp_img) == NULL)
        cerr << "No data available !\n";
    ImgData tmp_img;
    while(1){
        char filename[50];
        if(fscanf(_fp_img, "%lf, %s", &tmp_img._stamp, filename) == EOF)
            break;
        // fscanf(_fp_img, "%lf, %s", &stamp, filename);
        // tmp_img._stamp = stamp;
        tmp_img._stamp /= 1e9;//ns->s
        tmp_img._filename = filename;
        _img_data.push(tmp_img);
        count++;
    }
    ROS_INFO("Totally load %d images .", count);
    fclose(_fp_img);
    // ------------------- 读取IMU数据 ----------------- //    
    if(fgets(fileheader, 200, _fp_imu) == NULL)
        cerr << "No data available !\n";
    ImuData tmp_imu;
    count = 0;
    while(1){
        int end_flag;
        end_flag = fscanf(_fp_imu, "%lf, %f, %f, %f, %f, %f, %f", &tmp_imu._stamp,
                        &tmp_imu._gyr_x, &tmp_imu._gyr_y, &tmp_imu._gyr_z,
                        &tmp_imu._acc_x, &tmp_imu._acc_y, &tmp_imu._acc_z);
        if(end_flag == EOF)
            break;
        tmp_imu._stamp /= 1e9;//ns->s
        _imu_data.push(tmp_imu);
        count++;
    }
    ROS_INFO("Totally load %d imu measurements.", count);
    fclose(_fp_imu);
    return true;
}

void EurocDataset::pubImgMsg(){
    sensor_msgs::ImagePtr img_msg;
    string filepath = _img_data.front()._filename;
    ros::Time img_stamp(_img_data.front()._stamp);
    // 构建消息头
    std_msgs::Header img_header;
    img_header.seq = _img_seq++;
    img_header.stamp = img_stamp;
    img_header.frame_id = "cam0";
    // 构建消息数据
    cv::Mat img;
    img = cv::imread(_cam_path + "/data/" + filepath, CV_LOAD_IMAGE_GRAYSCALE);
    img_msg = cv_bridge::CvImage(img_header, "mono8", img).toImageMsg();
    //发布并弹出已发布的img消息
    _pub_img.publish(img_msg); 
    _img_data.pop();
}

void EurocDataset::pubImuMsg(){
    sensor_msgs::Imu imu_msg;
    ros::Time imu_stamp(_imu_data.front()._stamp);
    //构建消息头
    imu_msg.header.seq = _imu_seq++;
    imu_msg.header.stamp = imu_stamp;
    imu_msg.header.frame_id = "imu4";
    //构建量测消息
    imu_msg.linear_acceleration.x = _imu_data.front()._acc_x;
    imu_msg.linear_acceleration.y = _imu_data.front()._acc_y;
    imu_msg.linear_acceleration.z = _imu_data.front()._acc_z;
    
    imu_msg.angular_velocity.x = _imu_data.front()._gyr_x;
    imu_msg.angular_velocity.y = _imu_data.front()._gyr_y;
    imu_msg.angular_velocity.z = _imu_data.front()._gyr_z;
    //弹出已发布的imu消息   
    _pub_imu.publish(imu_msg);
    _imu_data.pop();
}