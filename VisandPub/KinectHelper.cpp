#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

#include "KinectHelper.h"

using namespace std;
using namespace cv;

bool KinectHelper::is_shutdown = false;

KinectHelper::KinectHelper(Processor depthProcessor) :
        pipeline(nullptr),
        dev(nullptr),
        registration(nullptr),
        listener(nullptr),
        updateThread(nullptr),
        nan(std::numeric_limits<float>::quiet_NaN())
{
    if (freenect2.enumerateDevices() == 0) {
        throw runtime_error("no device connected");
    }

    string serial = freenect2.getDefaultDeviceSerialNumber();

    std::cout << "SERIAL: " << serial << std::endl;

    if (depthProcessor == Processor::cpu) {
        if (!pipeline) {
            pipeline = new libfreenect2::CpuPacketPipeline();
        }
    } else if (depthProcessor == Processor::gl) {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        if (!pipeline) {
            pipeline = new libfreenect2::OpenGLPacketPipeline();
        }
#else
        std::cout << "OpenGL pipeline is not supported!" << std::endl;
#endif
    } else if (depthProcessor == Processor::cl) {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        if(!pipeline) {
            pipeline = new libfreenect2::OpenCLPacketPipeline();
        }
#else
        std::cout << "OpenCL pipeline is not supported!" << std::endl;
#endif
    }

    if (pipeline) {
        dev = freenect2.openDevice(serial, pipeline);
    } else {
        dev = freenect2.openDevice(serial);
    }

    if (dev == 0) {
        throw runtime_error("failure opening device");
    }

    listener = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color |
                                                          libfreenect2::Frame::Depth |
                                                          libfreenect2::Frame::Ir);

    dev->setColorFrameListener(listener);
    dev->setIrAndDepthFrameListener(listener);


    dev->start();


    libfreenect2::Freenect2Device::ColorCameraParams rgb =dev->getColorCameraParams();
    libfreenect2::Freenect2Device::IrCameraParams ip =dev->getIrCameraParams();



        std::cout << "rgb fx=" << rgb.fx << ",fy=" << rgb.fy <<
        ",cx=" << rgb.cx << ",cy=" << rgb.cy << std::endl;



        std::cout << "ir fx=" << ip.fx << ",fy=" << ip.fy <<
        ",cx=" << ip.cx << ",cy=" << ip.cy <<
        ",k1=" << ip.k1 << ",k2=" << ip.k2 << ",k3=" << ip.k3 <<
        ",p1=" << ip.p1 << ",p2=" << ip.p2 << std::endl;




    for(int i=0;i<512;i++)
    {
        colmap[i]=(i-ip.cx+0.5) /ip.fx;
    }

    for(int j =0;j<424;j++)
    {
        rowmap[j]=(j-ip.cy+0.5) /ip.fy;
    }

    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    registration = new libfreenect2::Registration(dev->getIrCameraParams(),
                                                    dev->getColorCameraParams());




    header_color.frame_id = "kinect2_rgb_optical_frame";
    header_depth.frame_id = "kinect2_ir_optical_frame";
    header_cloud.frame_id = "kinect2_rgb_optical_frame";

    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/kinect2/hd/points", 1);
    color_pub = nh.advertise<sensor_msgs::Image>("/kinect2/hd/image_color", 1);
    depth_pub = nh.advertise<sensor_msgs::Image>("/kinect2/hd/image_depth", 1);


}


void KinectHelper::trampoline( KinectHelper* KinectHelper)
{
    KinectHelper->updateMat();
}



void KinectHelper::updateMat()
{
    libfreenect2::FrameMap frames;

    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4),bigdepth(1920,1082,4);
    while (!is_shutdown)
    {

        listener->waitForNewFrame(frames);

        std::unique_lock<std::mutex> lock(updateMutex);

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        cv::Mat tmp_rgbMat(rgb->height,rgb->width,CV_8UC4,rgb->data);
        cv::flip(tmp_rgbMat,rgbMat,1);

        registration->apply(rgb, depth, &undistorted, &registered, true,&bigdepth);

        cv::Mat depthMatUndistortedtmp(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
        depthMatUndistortedtmp /= 4096.0f;

        cv::Mat rgbdMattmp(registered.height, registered.width, CV_8UC4, registered.data);

        cv::Mat bigdepthtmp(bigdepth.height, bigdepth.width, CV_8UC4,bigdepth.data);
       // bigdepthtmp /=4096.0f;

        cv::flip(rgbdMattmp,rgbdMat,1);
        cv::flip(depthMatUndistortedtmp,depthMatUndistorted,1);
        cv::flip(bigdepthtmp,Bigdepth,1);

        listener->release(frames);

        lock.unlock();

        initSig.notify_all();
    }
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr KinectHelper::getcloud()
{

    const short w =(this->getDepthMatUndistorted()).cols;
    const short h =(this->getDepthMatUndistorted()).rows;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));

    if(cloud->size()!=w*h)
        cloud->resize(w*h);

    const float * itD0 = (float *) (this->getDepthMatUndistorted()).ptr();
    const char * itRGB0 = (char *)(this->getRGBd()).ptr();

    pcl::PointXYZRGB * itP = &cloud->points[0];
    bool is_dense = true;

    for(std::size_t y = 0; y < h; ++y){

        const unsigned int offset = y * w;
        const float * itD = itD0 + offset;
        const char * itRGB = itRGB0 + offset * 4;
        const float dy = rowmap[y];

        for(std::size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4 )
        {
              const float depth_value = *itD / 1000.0f;

              if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){

                      const float rx = colmap[x] * depth_value;
      const float ry = dy * depth_value;
                      itP->z = depth_value;
                      itP->x = rx;
                      itP->y = ry;

                      itP->b = itRGB[0];
                      itP->g = itRGB[1];
                      itP->r = itRGB[2];
              } else {
                      itP->z = nan;
                      itP->x = nan;
                      itP->y = nan;

                      itP->b = nan;
                      itP->g = nan;
                      itP->r = nan;
                      is_dense = false;
              }
        }
    }
    cloud->is_dense = is_dense;
    return cloud;

}


void KinectHelper::stop()
{
    is_shutdown = true;

    updateThread->join();
}

void KinectHelper::start()
{
    updateThread = new std::thread( trampoline, this );

    std::unique_lock<std::mutex> lock(updateMutex);
    initSig.wait(lock);  // unlock when block and lock when not blocked any more,block the main thread
}



KinectHelper::~KinectHelper()
{
    dev->stop();

    delete registration;
    delete updateThread;
}





void KinectHelper::createImage(cv::Mat & image, std_msgs::Header & header, sensor_msgs::Image & msgImage) const
{
        size_t step, size;
        step = image.cols * image.elemSize();
        size = image.rows * step;
        msgImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

        msgImage.header = header;
        msgImage.height = image.rows;
        msgImage.width = image.cols;
        msgImage.is_bigendian = false;
        msgImage.step = step;
        msgImage.data.resize(size);

        memcpy(&msgImage.data[0], image.data, size);
}


void KinectHelper::publishAll()
{

        cv::Mat tmp_depth=this->getBigdepth();
        cv::Mat tmp_color=this->getRGB();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =this->getcloud();


        header_depth.stamp = ros::Time::now();
        createImage(tmp_depth, header_depth, depth_image);
        depth_pub.publish(depth_image);



        header_color.stamp = ros::Time::now();
        color_image = cv_bridge::CvImage(header_color, "bgra8", tmp_color).toImageMsg();
        color_pub.publish(color_image);



        pcl::toROSMsg(*cloud, point_cloud_2);
        point_cloud_2.header.frame_id = "world";
        point_cloud_2.header.stamp = ros::Time::now();
        point_cloud_pub.publish(point_cloud_2);
}

