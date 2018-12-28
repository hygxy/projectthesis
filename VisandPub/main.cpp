#include "KinectHelper.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>


#include <unistd.h>
#include <cmath>
#include <chrono>

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;


int main(int argc, char** argv)
{

       ros::init(argc, argv, "KinectHelper");

        KinectHelper kinecthelper;
        int key = 0;


        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;


        // starts the class thread gathering pipline info from
        // the kinect2

        kinecthelper.start();
        cloud =kinecthelper.getcloud();

        cloud->sensor_orientation_.w() = 0.0;
        cloud->sensor_orientation_.x() = 1.0;
        cloud->sensor_orientation_.y() = 0.0;
        cloud->sensor_orientation_.z() = 0.0;

        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");



        while ((ros::ok()) && key <= 0&& (!viewer->wasStopped())) {

            // grab the current RGB matrix
          //  cv::Mat img = libfreenect2OpenCV.getRGB();
          //  cv::Mat edges = img;

            // opencv to gray scale
          //  cv::cvtColor(edges, edges, CV_BGR2GRAY);
          //  cv::Canny(edges, edges, 120, 175);

            // Show canny edges
          //  cv::imshow("Canny Edges", edges);

            // Show original RGB Image
          //  cv::imshow("RGB Matrix", img);

            // Show depth matrix
        //   cv::imshow("Depth Matrix", kinecthelper.getBigdepth());

            kinecthelper.publishAll();

            cloud =kinecthelper.getcloud();

           // imshow("bigdepth",kinecthelper.getBigdepth());

            viewer->spinOnce ();
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
            viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

            key = cv::waitKey(1);
        }
    kinecthelper.stop();
}


