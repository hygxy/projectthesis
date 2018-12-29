#include <ros/ros.h>
#include<ObjectDetection/Location.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

int centerX;
int centerY;

class SynSubscriber
{
public:
    SynSubscriber(const Mat &src):
    it_(nh_),
    src_(src),
    color_sub_(it_, "/kinect2/hd/image_color", 1),
    depth_sub_(it_, "/kinect2/hd/image_depth", 1),
    sync(MySyncPolicy(10), color_sub_, depth_sub_)
    {
        sync.registerCallback( boost::bind( &SynSubscriber::callback, this, _1, _2 ) );
    }

    void callback(const sensor_msgs::ImageConstPtr& color_msg, const sensor_msgs::ImageConstPtr& depth_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
          cv_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }


        cv::Mat source =cv_ptr->image;

        cv::Mat model;
        cvtColor(this->src_, model,CV_BGR2GRAY);

        vector<KeyPoint> model_keypoints, scene_img_keypoints;
        cv::Mat model_descriptors, scene_img_descriptors;

        int minHessian =400;
        Ptr<SURF> detector =SURF::create(minHessian);
        detector->detectAndCompute( model, Mat(), model_keypoints, model_descriptors );

        FlannBasedMatcher matcher;

        Mat scene_img;
        cvtColor(source,scene_img,CV_BGR2GRAY);

        detector->detectAndCompute( scene_img, Mat(), scene_img_keypoints, scene_img_descriptors );
        vector<DMatch> matches;
        matcher.match(model_descriptors,scene_img_descriptors,matches);

        double min_dist =100, max_dist =0;
           for (int i =0; i<model_descriptors.rows;i++)
           {
               double dist =matches[i].distance;
               if (dist <min_dist) min_dist =dist;
               if (dist >max_dist) max_dist =dist;
           }

          vector<DMatch> good_matches;
           for (int i =0; i <model_descriptors.rows;i++)
           {
              if(matches[i].distance<=max(2*min_dist,30.0))
              {
                 good_matches.push_back(matches[i]);
              }

            }

           Mat img_matches;
           drawMatches( model, model_keypoints, source, scene_img_keypoints,
                        good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                        std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

           vector<Point2f> obj;
           vector<Point2f> scene;
           for( size_t i = 0; i < good_matches.size(); i++ )
           {

             obj.push_back( model_keypoints[ good_matches[i].queryIdx ].pt );
             scene.push_back( scene_img_keypoints[ good_matches[i].trainIdx ].pt );
           }

           Mat H = findHomography( obj, scene, RANSAC );

           vector<Point2f> obj_corners(4);
           obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( model.cols, 0 );
           obj_corners[2] = cvPoint( model.cols, model.rows ); obj_corners[3] = cvPoint( 0, model.rows );
           vector<Point2f> scene_corners(4);
           perspectiveTransform( obj_corners, scene_corners, H);

           line( img_matches, scene_corners[0] + Point2f( model.cols, 0), scene_corners[1] + Point2f( model.cols, 0), Scalar(255, 0, 0), 4 );
           line( img_matches, scene_corners[1] + Point2f( model.cols, 0), scene_corners[2] + Point2f( model.cols, 0), Scalar( 0, 255, 0), 4 );
           line( img_matches, scene_corners[2] + Point2f( model.cols, 0), scene_corners[3] + Point2f( model.cols, 0), Scalar( 0, 0, 255), 4 );
           line( img_matches, scene_corners[3] + Point2f( model.cols, 0), scene_corners[0] + Point2f( model.cols, 0), Scalar( 0, 0, 0), 4 );


           centerX =(int)((scene_corners[2].x+scene_corners[0].x)/2);
           centerY =(int)((scene_corners[2].y+scene_corners[0].y)/2);



        imshow( "Good Matches & Object detection", img_matches );



        cv::imshow(OPENCV_WINDOW, source);
        usleep(100000);
        cv::waitKey(1);
        cv_bridge::CvImagePtr cv_ptr_depth;
        try
        {
          cv_ptr_depth = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

         location_pub =nh_.advertise<ObjectDetection::Location>("Location",1);
         ObjectDetection::Location l;
         l.x =((float)(centerX-959.5)*(cv_ptr_depth->image.at<float>(centerX,centerY)/1000.0f))/(1081.37);
         l.y =((float)(centerY+1-539.5)*(cv_ptr_depth->image.at<float>(centerX,centerY)/1000.0f))/(1081.37);
         l.z =(cv_ptr_depth->image.at<float>(centerX,centerY)/1000.0f);
         location_pub.publish(l);
      }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::SubscriberFilter color_sub_;
    image_transport::SubscriberFilter depth_sub_;
    ros::Publisher location_pub;
    typedef message_filters::sync_policies::ApproximateTime<
       sensor_msgs::Image, sensor_msgs::Image
     > MySyncPolicy;

     message_filters::Synchronizer< MySyncPolicy > sync;
     Mat src_;
};


int main(int argc, char** argv)
{
  Mat src =imread(argv[1],1);
  ros::init(argc, argv, "imagesubscriber");

  SynSubscriber ss(src);
  while(ros::ok())
  {
       ros::spin();
  }

  return 0;
}


