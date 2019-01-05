#include "KinectHelper.h"
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv)
{

       ros::init(argc, argv, "KinectHelper");

        KinectHelper kinecthelper;

        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;

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


        while ((ros::ok()) && (!viewer->wasStopped())) {

            kinecthelper.publishAll();

            cloud =kinecthelper.getcloud();

            viewer->spinOnce ();
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
            viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");

        }
    kinecthelper.stop();
}


