/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include "semantic_interfaces/msg/detection2_d_array.hpp"
#include "semantic_interfaces/msg/detection2_d.hpp"
#include "semantic_interfaces/msg/instance_mask.hpp"

Estimator estimator;

queue<sensor_msgs::msg::Imu::ConstPtr> imu_buf;
queue<sensor_msgs::msg::PointCloud::ConstPtr> feature_buf;
queue<sensor_msgs::msg::Image::ConstPtr> img0_buf;
queue<sensor_msgs::msg::Image::ConstPtr> img1_buf;
std::mutex m_buf;

// header: 1403715278
void img0_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    m_buf.lock();
    // std::cout << "Left : " << img_msg->header.stamp.sec << "." << img_msg->header.stamp.nanosec << endl;
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    m_buf.lock();
    // std::cout << "Right: " << img_msg->header.stamp.sec << "." << img_msg->header.stamp.nanosec << endl;
    img1_buf.push(img_msg);
    m_buf.unlock();
}


// cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::SharedPtr img_msg)
cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                double time1 = img1_buf.front()->header.stamp.sec + img1_buf.front()->header.stamp.nanosec * (1e-9);

                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");

                    // std::cout << std::fixed << img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9) << std::endl;
                    // assert(0);
                    
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    // std::cout << "IMU cb" << std::endl;

    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * (1e-9);
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);

    // std::cout << "got t_imu: " << std::fixed << t << endl;
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
{
    std::cout << "feature cb" << std::endl;
    std::cout << "Feature: " << feature_msg->points.size() << std::endl;


    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        assert(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.sec + feature_msg->header.stamp.nanosec * (1e-9);
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::msg::Bool::SharedPtr switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::msg::Bool::SharedPtr switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

// docs/05 动态物体特征点剔除：订阅 /yolo/detections，把动态类的实例 mask 合并成一张
// 0/1 二值图交给 estimator；estimator 在光流跟踪后用它剔除落在动态 mask 内的特征点。
void yolo_callback(const semantic_interfaces::msg::Detection2DArray::SharedPtr det_msg)
{
    if (DYNAMIC_CLASSES.empty())
        return;

    cv::Mat merged;      // 合并后的动态类二值图（0/255）
    int ref_w = -1, ref_h = -1;
    for (const auto &d : det_msg->detections)
    {
        bool is_dynamic = false;
        for (int c : DYNAMIC_CLASSES)
            if (d.class_id == c) { is_dynamic = true; break; }
        if (!is_dynamic)
            continue;

        const auto &m = d.mask;
        if (m.width <= 0 || m.height <= 0 || (int)m.data.size() != m.width * m.height)
            continue;

        // 消息里的 mask 是 0/1 行主序 uint8[]，包成 cv::Mat（视图，仅在本次循环内使用）
        cv::Mat inst(m.height, m.width, CV_8UC1, (void *)m.data.data());
        if (merged.empty())
        {
            ref_w = m.width; ref_h = m.height;
            merged = cv::Mat(ref_h, ref_w, CV_8UC1, cv::Scalar(0));
        }
        if (m.width != ref_w || m.height != ref_h)   // 各实例 mask 尺寸不一致时就近对齐
            cv::resize(inst, inst, cv::Size(ref_w, ref_h), 0, 0, cv::INTER_NEAREST);
        cv::Mat dyn = inst > 0;                       // 0/1 -> 0/255
        cv::bitwise_or(merged, dyn, merged);
    }
    if (merged.empty())
        return;

    double t = det_msg->header.stamp.sec + det_msg->header.stamp.nanosec * (1e-9);
    estimator.inputDynamicMask(t, merged);
    ROS_INFO("yolo det: %zu dets, dynamic mask %dx%d", det_msg->detections.size(),
             merged.cols, merged.rows);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
	auto n = rclcpp::Node::make_shared("vins_estimator");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);


    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu = NULL;
    if(USE_IMU)
    {
        sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort(), imu_callback);
    }
    auto sub_feature = n->create_subscription<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", rclcpp::QoS(rclcpp::KeepLast(2000)), feature_callback);
    auto sub_img0 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE0_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img0_callback);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img1 = NULL;
    if(STEREO)
    {
        sub_img1 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE1_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img1_callback);
    }
    
    auto sub_restart = n->create_subscription<std_msgs::msg::Bool>("/vins_restart", rclcpp::QoS(rclcpp::KeepLast(100)), restart_callback);
    // docs/05 动态物体特征点剔除：订阅 YOLO 检测（best_effort 对齐 yolo_seg_node 的 QoS）
    auto sub_yolo = n->create_subscription<semantic_interfaces::msg::Detection2DArray>(
        "/yolo/detections", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(), yolo_callback);
    auto sub_imu_switch = n->create_subscription<std_msgs::msg::Bool>("/vins_imu_switch", rclcpp::QoS(rclcpp::KeepLast(100)), imu_switch_callback);
    auto sub_cam_switch = n->create_subscription<std_msgs::msg::Bool>("/vins_cam_switch", rclcpp::QoS(rclcpp::KeepLast(100)), cam_switch_callback);

    std::thread sync_thread{sync_process};
    rclcpp::spin(n);

    return 0;
}
