#include "hitll.h"

void HITLL::loadDataset(){
    // check file point states
    if(_fp_mynt_img == nullptr){
        ROS_WARN("Failed to open mynt image file!"); 
        return;
    }
    if(_fp_mynt_imu == nullptr){
        ROS_WARN("Failed to open mynt image file!");
        return;
    }
    if(_fp_mynt_img == nullptr){
        ROS_WARN("Failed to open mynt image file!");
        return;
    }
    if(_fp_mynt_img == nullptr){
        ROS_WARN("Failed to open mynt image file!");
        return;
    }
    if(_fp_mynt_img == nullptr){
        ROS_WARN("Failed to open mynt image file!");
        return;
    }

    int count = 0;
    ImgData tmp_img;
    ImuData tmp_imu;
    PoseData tmp_pose;
    //-------------------- 读取Mynteye图像数据 ----------------- //
    char fileheader[200];
    if(fgets(fileheader, 200, _fp_mynt_img) == NULL)
        cerr << "No Mynteye img data available !\n";
    while(1){
        char filename[50];
        if(fscanf(_fp_mynt_img, "%lf, %s", &tmp_img._stamp, filename) == EOF)
            break;
        tmp_img._stamp /= 1e9;//ns->s
        tmp_img._filename = filename;
        _mynt_img_data.push(tmp_img);
        count++;
    }
    ROS_INFO("Totally load %d images from Mynteye sequence.", count);
    // ------------------- 读取Mynteye IMU数据 ----------------- //    
    if(fgets(fileheader, 200, _fp_mynt_imu) == NULL)
        cerr << "No Mynteye imu data available !\n";
    count = 0;
    while(1){
        int end_flag;
        end_flag = fscanf(_fp_mynt_imu, "%lf, %f, %f, %f, %f, %f, %f", &tmp_imu._stamp,
                        &tmp_imu._gyr_x, &tmp_imu._gyr_y, &tmp_imu._gyr_z,
                        &tmp_imu._acc_x, &tmp_imu._acc_y, &tmp_imu._acc_z);
        if(end_flag == EOF)
            break;
        tmp_imu._stamp /= 1e9;//ns->s
        _mynt_imu_data.push(tmp_imu);
        count++;
    }
    ROS_INFO("Totally load %d imu measurements from Mynteye sequence.", count);
    // ------------------- 读取Pixhawk IMU数据 ----------------- //    
    if(fgets(fileheader, 200, _fp_pix_imu) == NULL)
        cerr << "No Pixhawk imu data available !\n";
    count = 0;
    while(1){
        int end_flag;
        end_flag = fscanf(_fp_pix_imu, "%lf, %f, %f, %f, %f, %f, %f", &tmp_imu._stamp,
                        &tmp_imu._gyr_x, &tmp_imu._gyr_y, &tmp_imu._gyr_z,
                        &tmp_imu._acc_x, &tmp_imu._acc_y, &tmp_imu._acc_z);
        if(end_flag == EOF)
            break;
        tmp_imu._stamp /= 1e9;//ns->s
        _pix_imu_data.push(tmp_imu);
        count++;
    }
    ROS_INFO("Totally load %d imu measurements from Pixhawk sequence.", count);
    // ------------------- 读取Pixhawk pose数据 ----------------- //    
    if(fgets(fileheader, 200, _fp_pix_pose) == NULL)
        cerr << "No Pixhawk pose data available !\n";
    count = 0;
    while(1){
        int end_flag;
        end_flag = fscanf(_fp_pix_pose, "%lf, %f, %f, %f, %f, %f, %f, %f", &tmp_pose._stamp,
                        &tmp_pose._pos_x, &tmp_pose._pos_y, &tmp_pose._pos_z,
                        &tmp_pose._ori_w, &tmp_pose._ori_x, &tmp_pose._ori_y, &tmp_pose._ori_z);
        if(end_flag == EOF)
            break;
        tmp_pose._stamp /= 1e9;//ns->s
        _pix_pose_data.push(tmp_pose);
        count++;
    }
    ROS_INFO("Totally load %d pose measurements from Pixhawk sequence.", count);
    // ------------------- 读取Optitrack pose数据 ----------------- //    
    if(fgets(fileheader, 200, _fp_opt_pose) == NULL)
        cerr << "No Optitrack pose data available !\n";
    count = 0;
    while(1){
        int end_flag;
        end_flag = fscanf(_fp_opt_pose, "%lf, %f, %f, %f, %f, %f, %f, %f", &tmp_pose._stamp,
                        &tmp_pose._pos_x, &tmp_pose._pos_y, &tmp_pose._pos_z,
                        &tmp_pose._ori_w, &tmp_pose._ori_x, &tmp_pose._ori_y, &tmp_pose._ori_z);
        if(end_flag == EOF)
            break;
        tmp_pose._stamp /= 1e9;//ns->s
        _opt_pose_data.push(tmp_pose);
        count++;
    }
    ROS_INFO("Totally load %d pose measurements from Optitrack sequence.", count);
}

void HITLL::pubMyntImg(){
    sensor_msgs::ImagePtr img_msg;
    string filepath = _mynt_img_data.front()._filename;
    ros::Time img_stamp(_mynt_img_data.front()._stamp);
    // 构建消息头
    std_msgs::Header img_header;
    img_header.seq = _mynt_img_count++;
    img_header.stamp = img_stamp;
    img_header.frame_id = "cam0";
    // 构建消息数据
    cv::Mat img;
    img = cv::imread(_data_path + "/mynteye/cam0/data/" + filepath, CV_LOAD_IMAGE_GRAYSCALE);
    img_msg = cv_bridge::CvImage(img_header, "mono8", img).toImageMsg();
    //发布并弹出已发布的img消息
    _mynt_img_pub.publish(img_msg); 
    _mynt_img_data.pop();
}

void HITLL::pubMyntImu(){
    sensor_msgs::Imu imu_msg;
    ros::Time imu_stamp(_mynt_imu_data.front()._stamp);
    //构建消息头
    imu_msg.header.seq = _mynt_imu_count++;
    imu_msg.header.stamp = imu_stamp;
    imu_msg.header.frame_id = "imu4";
    //构建量测消息
    imu_msg.linear_acceleration.x = _mynt_imu_data.front()._acc_x;
    imu_msg.linear_acceleration.y = _mynt_imu_data.front()._acc_y;
    imu_msg.linear_acceleration.z = _mynt_imu_data.front()._acc_z;
    
    imu_msg.angular_velocity.x = _mynt_imu_data.front()._gyr_x;
    imu_msg.angular_velocity.y = _mynt_imu_data.front()._gyr_y;
    imu_msg.angular_velocity.z = _mynt_imu_data.front()._gyr_z;
    //弹出已发布的imu消息   
    _mynt_imu_pub.publish(imu_msg);
    _mynt_imu_data.pop();
}

void HITLL::pubPixImu(){
    sensor_msgs::Imu imu_msg;
    ros::Time imu_stamp(_pix_imu_data.front()._stamp);
    //构建消息头
    imu_msg.header.seq = _pix_imu_count++;
    imu_msg.header.stamp = imu_stamp;
    imu_msg.header.frame_id = "imu0";
    //构建量测消息
    imu_msg.linear_acceleration.x = _pix_imu_data.front()._acc_x;
    imu_msg.linear_acceleration.y = _pix_imu_data.front()._acc_y;
    imu_msg.linear_acceleration.z = _pix_imu_data.front()._acc_z;
    
    imu_msg.angular_velocity.x = _pix_imu_data.front()._gyr_x;
    imu_msg.angular_velocity.y = _pix_imu_data.front()._gyr_y;
    imu_msg.angular_velocity.z = _pix_imu_data.front()._gyr_z;
    //弹出已发布的imu消息   
    _pix_imu_pub.publish(imu_msg);
    _pix_imu_data.pop();
}

void HITLL::pubPixPath(){
}

void HITLL::pubOptPath(){
    // 消息头
    _opt_path_ptr->header.stamp = ros::Time(_opt_pose_data.front()._stamp);
    _opt_path_ptr->header.frame_id = "map";
    _opt_path_ptr->header.seq = _opt_pose_count++;
    // 添加当前位姿
    geometry_msgs::PoseStamped curr_pose;
    curr_pose.header.stamp = ros::Time(_opt_pose_data.front()._stamp);
    curr_pose.header.frame_id = "map";
    
    curr_pose.pose.position.x = _opt_pose_data.front()._pos_x; 
    curr_pose.pose.position.y = _opt_pose_data.front()._pos_y; 
    curr_pose.pose.position.z = _opt_pose_data.front()._pos_z; 
    
    curr_pose.pose.orientation.w = _opt_pose_data.front()._ori_w;
    curr_pose.pose.orientation.x = _opt_pose_data.front()._ori_x;
    curr_pose.pose.orientation.y = _opt_pose_data.front()._ori_y;
    curr_pose.pose.orientation.z = _opt_pose_data.front()._ori_z;
    
    _opt_path_ptr->poses.push_back(curr_pose);
    // 发布 path
    _opt_path_pub.publish(_opt_path_ptr);
    _opt_pose_data.pop();
}