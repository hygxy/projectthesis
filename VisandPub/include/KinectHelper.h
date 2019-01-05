#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
#include <limits>
#include <unistd.h>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>


class KinectHelper {

    libfreenect2::PacketPipeline *pipeline;
    libfreenect2::Freenect2Device *dev;
    libfreenect2::Registration *registration;
    libfreenect2::SyncMultiFrameListener *listener;
    libfreenect2::Freenect2 freenect2;

    cv::Mat depthMatUndistorted;
    cv::Mat rgbdMat;
    cv::Mat rgbMat;
    cv::Mat Bigdepth;


    ros::NodeHandle nh;
    ros::Publisher point_cloud_pub, color_pub, depth_pub;
    sensor_msgs::PointCloud2 point_cloud_2;
    std_msgs::Header header_depth, header_color, header_cloud;
    sensor_msgs::Image depth_image;
    sensor_msgs::Image::Ptr color_image;




    std::thread * updateThread;
    std::mutex updateMutex;
    std::condition_variable initSig;

    float colmap[512];
    float rowmap[424];
    float nan;

    static void trampoline(KinectHelper * );
    void updateMat();

    void createImage(cv::Mat & image, std_msgs::Header & header, sensor_msgs::Image & msgImage) const;


    enum Processor {
        cl, gl, cpu
    };

protected:
    static bool is_shutdown;

public:
    KinectHelper (Processor depthProcessor = Processor::gl);
    virtual ~KinectHelper();



    void start();
    void stop();




    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getcloud();


    void publishAll();

    const cv::Mat & getBigdepth()
    {
        std::unique_lock<std::mutex> lock(updateMutex);
        return Bigdepth;
    }


    const cv::Mat & getRGB()
    {
        std::unique_lock<std::mutex> lock(updateMutex);
        return rgbMat;
    }

    const cv::Mat & getDepthMatUndistorted()
    {
        std::unique_lock<std::mutex> lock(updateMutex);
        return depthMatUndistorted;
    }

    const cv::Mat & getRGBd()
    {
        std::unique_lock<std::mutex> lock(updateMutex);
        return rgbdMat;
    }

};


