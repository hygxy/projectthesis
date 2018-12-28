#include "Libfreenect2OpenCV.h"
#include <opencv2/opencv.hpp>
#include <unistd.h>

using namespace cv;
using namespace std;
int main()
{
    libfreenect2opencv::Libfreenect2OpenCV libfreenect2OpenCV;
    int key = 0;

    // starts the class thread gathering pipline info from
    // the kinect2

    libfreenect2OpenCV.start();

  //  cv::imwrite("rgb4.png",libfreenect2OpenCV.getRGBMat());


    Mat src =libfreenect2OpenCV.getRGBMat();   //getRGBMat() returns Mat of CV_8UC4 type
    Mat rgb;
    cvtColor(src,rgb,CV_BGRA2BGR);

    Mat depth =libfreenect2OpenCV.getRGBd2();   // big_depth


    Size pattersize =Size(9,7);
    vector<Point2f> corners;
    bool found =findChessboardCorners(rgb,pattersize,corners);
    if(found)
    {
        Mat viewGray;
        cvtColor(rgb,viewGray,CV_BGR2GRAY);
        cornerSubPix(viewGray,corners,Size(5,5),Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
    }

    cout <<" the state of found is :" <<found <<endl;



  //  drawChessboardCorners(rgb,pattersize,corners,found);
    imshow("rgb",rgb);


            double SquareDistanceMeters =2.0 /100 ;// 2.0cm -> 0.02m
            vector<Point3f> objPoints;
            for(int i =0; i<pattersize.height;i++)        //heigt =7
            {
                for(int j =0; j<pattersize.width;j++)     // width =9
                    objPoints.push_back(Point3f(j*SquareDistanceMeters,i*SquareDistanceMeters,0));
            }

            vector<KeyPoint> keypoints;
            for(size_t i =0; i<corners.size();i++)
            {
                keypoints.push_back(KeyPoint(corners[i],4.f));
                cout <<"image points :" <<"("<<corners[i].x<<","<<corners[i].y<<")   "<<"depth is :"<<
                depth.at<float>(corners[i].y,corners[i].x)<<"   object points :"<<"("<<objPoints[i].x<<","
                <<objPoints[i].y<<","<<objPoints[i].z<<")"<<endl;
            }

            // use opencv coordinate system cenvention, i.e., Y points downward and X points upward
/*
            sample data set during one test: (upper left is world origin)

            image points :(725.959,694.112)   depth is :1441.55   object points :(0,0,0)
            image points :(744.157,694.094)   depth is :1442.14   object points :(0.02,0,0)
            image points :(761.942,694.499)   depth is :1439.82   object points :(0.04,0,0)
            image points :(780.095,694.638)   depth is :1442.17   object points :(0.06,0,0)
            image points :(797.926,694.964)   depth is :1442.4   object points :(0.08,0,0)
            image points :(816.073,695.034)   depth is :1444.51   object points :(0.1,0,0)
            image points :(833.725,695.21)   depth is :1444.3   object points :(0.12,0,0)
            image points :(851.668,695.387)   depth is :1446.36   object points :(0.14,0,0)
            image points :(869.56,695.59)   depth is :1446.91   object points :(0.16,0,0)
            image points :(725.295,710.988)   depth is :1435.81   object points :(0,0.02,0)
            image points :(743.583,711.313)   depth is :1438.13   object points :(0.02,0.02,0)
            image points :(761.329,711.425)   depth is :1439.57   object points :(0.04,0.02,0)
            image points :(779.653,711.64)   depth is :1439.77   object points :(0.06,0.02,0)
            image points :(797.418,711.736)   depth is :1439.93   object points :(0.08,0.02,0)
            image points :(815.558,711.941)   depth is :1442.38   object points :(0.1,0.02,0)
            image points :(833.379,712.118)   depth is :1444.65   object points :(0.12,0.02,0)
            image points :(851.28,712.265)   depth is :1445.78   object points :(0.14,0.02,0)
            image points :(869.23,712.417)   depth is :1449.51   object points :(0.16,0.02,0)
            image points :(724.64,728.066)   depth is :1434.81   object points :(0,0.04,0)
            image points :(742.891,728.167)   depth is :1436.07   object points :(0.02,0.04,0)
            image points :(760.852,728.35)   depth is :1437.8   object points :(0.04,0.04,0)
            image points :(779.16,728.529)   depth is :1439.5   object points :(0.06,0.04,0)
            image points :(796.977,728.62)   depth is :1439.69   object points :(0.08,0.04,0)
            image points :(815.21,728.68)   depth is :1440.19   object points :(0.1,0.04,0)
            image points :(833.038,728.814)   depth is :1442.12   object points :(0.12,0.04,0)
            image points :(850.939,728.936)   depth is :1444.88   object points :(0.14,0.04,0)
            image points :(868.909,729.022)   depth is :1444.79   object points :(0.16,0.04,0)
            image points :(723.971,744.755)   depth is :1433.93   object points :(0,0.06,0)
            image points :(742.382,745.089)   depth is :1432.82   object points :(0.02,0.06,0)
            image points :(760.43,745.06)   depth is :1436.73   object points :(0.04,0.06,0)
            image points :(778.62,745.289)   depth is :1434.89   object points :(0.06,0.06,0)
            image points :(796.606,745.334)   depth is :1438.26   object points :(0.08,0.06,0)
            image points :(814.678,745.455)   depth is :1437.8   object points :(0.1,0.06,0)
            image points :(832.645,745.49)   depth is :1440.77   object points :(0.12,0.06,0)
            image points :(850.56,745.596)   depth is :1442.41   object points :(0.14,0.06,0)
            image points :(868.588,745.606)   depth is :1444.39   object points :(0.16,0.06,0)
            image points :(723.63,761.793)   depth is :1433.83   object points :(0,0.08,0)
            image points :(741.958,761.776)   depth is :1432.96   object points :(0.02,0.08,0)
            image points :(759.93,762.071)   depth is :1433.8   object points :(0.04,0.08,0)
            image points :(778.236,762.071)   depth is :1435.17   object points :(0.06,0.08,0)
            image points :(796.217,762.125)   depth is :1440.19   object points :(0.08,0.08,0)
            image points :(814.351,762.265)   depth is :1438.81   object points :(0.1,0.08,0)
            image points :(832.306,762.334)   depth is :1439.62   object points :(0.12,0.08,0)
            image points :(850.355,762.376)   depth is :1440.46   object points :(0.14,0.08,0)
            image points :(868.256,762.492)   depth is :1439.8   object points :(0.16,0.08,0)
            image points :(722.809,778.796)   depth is :1428.32   object points :(0,0.1,0)
            image points :(741.238,779.123)   depth is :1429.54   object points :(0.02,0.1,0)
            image points :(759.438,778.85)   depth is :1433.05   object points :(0.04,0.1,0)
            image points :(777.754,779.076)   depth is :1433.37   object points :(0.06,0.1,0)
            image points :(795.813,779.076)   depth is :1433.31   object points :(0.08,0.1,0)
            image points :(813.873,779.211)   depth is :1434.11   object points :(0.1,0.1,0)
            image points :(831.95,779.302)   depth is :1437.8   object points :(0.12,0.1,0)
            image points :(849.89,779.441)   depth is :1440.7   object points :(0.14,0.1,0)
            image points :(867.838,779.593)   depth is :1438.02   object points :(0.16,0.1,0)
            image points :(722.047,796.334)   depth is :1422.44   object points :(0,0.12,0)
            image points :(740.531,796.255)   depth is :1423.27   object points :(0.02,0.12,0)
            image points :(758.79,796.296)   depth is :1429.4   object points :(0.04,0.12,0)
            image points :(777.108,796.236)   depth is :1430.68   object points :(0.06,0.12,0)
            image points :(795.284,796.275)   depth is :1430.93   object points :(0.08,0.12,0)
            image points :(813.404,796.343)   depth is :1434.29   object points :(0.1,0.12,0)
            image points :(831.552,796.368)   depth is :1434.15   object points :(0.12,0.12,0)
            image points :(849.482,796.468)   depth is :1435.58   object points :(0.14,0.12,0)
            image points :(867.396,796.62)   depth is :1434.91   object points :(0.16,0.12,0)

            translation vector is :
            [-0.25861958;
             0.16971515;
             1.1915338]

            rotation matrix is :
            [0.9916659, -0.052554507, -0.11762974;
             0.030333918, 0.98259354, -0.18327516;
             0.12521416, 0.17817956, 0.9759987]




            (bottom right is wolrd origin):
            image points :(867.172,791.385)   depth is :1437.76   object points :(0,0,0)
            image points :(849.37,791.282)   depth is :1434.61   object points :(0.02,0,0)
            image points :(831.265,791.215)   depth is :1436.61   object points :(0.04,0,0)
            image points :(813.277,791.212)   depth is :1433.54   object points :(0.06,0,0)
            image points :(795.178,791.164)   depth is :1434.14   object points :(0.08,0,0)
            image points :(776.936,791.126)   depth is :1430   object points :(0.1,0,0)
            image points :(758.747,791.302)   depth is :1427.16   object points :(0.12,0,0)
            image points :(740.435,791.253)   depth is :1423.99   object points :(0.14,0,0)
            image points :(721.921,791.373)   depth is :1422.12   object points :(0.16,0,0)
            image points :(867.564,774.334)   depth is :1440.21   object points :(0,0.02,0)
            image points :(849.607,774.285)   depth is :1440.52   object points :(0.02,0.02,0)
            image points :(831.713,774.206)   depth is :1441.82   object points :(0.04,0.02,0)
            image points :(813.652,774.193)   depth is :1437.36   object points :(0.06,0.02,0)
            image points :(795.676,773.969)   depth is :1436.82   object points :(0.08,0.02,0)
            image points :(777.644,774.092)   depth is :1433.7   object points :(0.1,0.02,0)
            image points :(759.333,773.928)   depth is :1430.99   object points :(0.12,0.02,0)
            image points :(741.12,774.123)   depth is :1430.95   object points :(0.14,0.02,0)
            image points :(722.76,773.909)   depth is :1430.45   object points :(0.16,0.02,0)
            image points :(867.912,757.377)   depth is :1440.89   object points :(0,0.04,0)
            image points :(850.019,757.235)   depth is :1443.02   object points :(0.02,0.04,0)
            image points :(832.042,757.227)   depth is :1438.15   object points :(0.04,0.04,0)
            image points :(814.074,757.249)   depth is :1440.14   object points :(0.06,0.04,0)
            image points :(795.95,757.087)   depth is :1440.33   object points :(0.08,0.04,0)
            image points :(778.047,757.038)   depth is :1437.94   object points :(0.1,0.04,0)
            image points :(759.758,757.059)   depth is :1436.15   object points :(0.12,0.04,0)
            image points :(741.811,756.77)   depth is :1436.74   object points :(0.14,0.04,0)
            image points :(723.448,756.883)   depth is :1433.56   object points :(0.16,0.04,0)
            image points :(868.315,740.582)   depth is :1447.63   object points :(0,0.06,0)
            image points :(850.3,740.522)   depth is :1443.11   object points :(0.02,0.06,0)
            image points :(832.445,740.477)   depth is :1444.22   object points :(0.04,0.06,0)
            image points :(814.411,740.481)   depth is :1440.88   object points :(0.06,0.06,0)
            image points :(796.396,740.403)   depth is :1442.55   object points :(0.08,0.06,0)
            image points :(778.342,740.326)   depth is :1440.99   object points :(0.1,0.06,0)
            image points :(760.294,740.213)   depth is :1437.46   object points :(0.12,0.06,0)
            image points :(742.151,740.224)   depth is :1435.73   object points :(0.14,0.06,0)
            image points :(723.903,739.881)   depth is :1434.86   object points :(0.16,0.06,0)
            image points :(868.581,723.9)   depth is :1446.04   object points :(0,0.08,0)
            image points :(850.518,723.898)   depth is :1447.84   object points :(0.02,0.08,0)
            image points :(832.743,723.738)   depth is :1444.73   object points :(0.04,0.08,0)
            image points :(814.814,723.657)   depth is :1442.62   object points :(0.06,0.08,0)
            image points :(796.671,723.62)   depth is :1439.85   object points :(0.08,0.08,0)
            image points :(778.822,723.504)   depth is :1440.98   object points :(0.1,0.08,0)
            image points :(760.6,723.382)   depth is :1440.36   object points :(0.12,0.08,0)
            image points :(742.621,723.307)   depth is :1440.17   object points :(0.14,0.08,0)
            image points :(724.269,723.235)   depth is :1438.52   object points :(0.16,0.08,0)
            image points :(868.8,707.341)   depth is :1450.1   object points :(0,0.1,0)
            image points :(850.897,707.256)   depth is :1447.93   object points :(0.02,0.1,0)
            image points :(833.023,707.055)   depth is :1445.88   object points :(0.04,0.1,0)
            image points :(815.185,706.978)   depth is :1443.74   object points :(0.06,0.1,0)
            image points :(797.049,706.929)   depth is :1444.63   object points :(0.08,0.1,0)
            image points :(779.275,706.686)   depth is :1442.88   object points :(0.1,0.1,0)
            image points :(761.029,706.613)   depth is :1443.76   object points :(0.12,0.1,0)
            image points :(743.276,706.402)   depth is :1440.35   object points :(0.14,0.1,0)
            image points :(725.043,706.244)   depth is :1439.03   object points :(0.16,0.1,0)
            image points :(869.068,690.605)   depth is :1449.37   object points :(0,0.12,0)
            image points :(851.28,690.494)   depth is :1447.71   object points :(0.02,0.12,0)
            image points :(833.375,690.336)   depth is :1443.75   object points :(0.04,0.12,0)
            image points :(815.682,690.209)   depth is :1447.07   object points :(0.06,0.12,0)
            image points :(797.517,690.103)   depth is :1446.27   object points :(0.08,0.12,0)
            image points :(779.693,689.77)   depth is :1442.79   object points :(0.1,0.12,0)
            image points :(761.624,689.768)   depth is :1443.17   object points :(0.12,0.12,0)
            image points :(743.819,689.333)   depth is :1443.43   object points :(0.14,0.12,0)
            image points :(725.693,689.324)   depth is :1443.33   object points :(0.16,0.12,0)


            translation vector is :
            [-0.10667723;
             0.28692982;
             1.2347034]

            rotation matrix is :
            [-0.99165297, 0.051142283, -0.11835925;
             -0.028165514, -0.98172361, -0.18821655;
             -0.12582189, -0.18331185, 0.97496957]


*/
            Mat drawkp;
            drawKeypoints(rgb,keypoints,drawkp,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            vector<vector<Point3f> >objPointss(corners.size(),objPoints);  // corners.size() =63
          //  int w =rgb.cols;   // 1920
          //  int h =rgb.rows;   // 1080

            Mat rvec, tvec,R;
            Mat intrinsics =(Mat_<float>(3,3)<<1081.37,0,959.5,0,1081.37,539.5,0,0,1); // from libfreenect2
            // rgbinfo fx =1081.37, fy =1081.37, cx =959.5, cy =539.5

      //      Mat dist =(Mat_<float>(1,5)<<0.0864496,-0.269807,0.097056,0,0);
            solvePnP(objPoints,corners,intrinsics,noArray(),rvec, tvec);
            //Mat intrin,dist;
           // auto error =calibrateCamera(objPointss,corners,Size(w,h),intrin,dist,rvec,tvec);
            Rodrigues(rvec,R);

         //   cout <<" ----------------------------------" <<endl;
         //  cout << " Intrinsic parameters are :" << intrin <<endl;
         //   cout << "distorion are :" <<dist <<endl;
            cout << " Extrinsic parameters are : "<<" ----------------------------" <<endl;
            cout <<" translation vector is :" <<endl<<
                   tvec <<endl;
            cout <<" rotation matrix is :" <<endl<<
                   R <<endl;

            cout <<" -----------------------------------------------------------------"<<endl;

            Mat Rc =R.t();


            Mat C =-R.inv()*(tvec);

            cout <<"Camera's orientation with respect to world:"<<endl
                <<Rc <<endl;
            cout <<"---------------------------"<<endl;
            cout <<"location of camera-center in world coordinate system is:"<<endl
                <<C<<endl;


 /*           Let C be a column vector describing the location of the camera-center in world coordinates,
  *           and let Rc be the rotation matrix describing the camera's orientation with respect to the world coordinate axes.
  *           We have the following :
  *
  *           Rc =R.transpose()
  *           C =-R.inverse().dot(tvec)
  *
  *           for the data set(upper left is world origin) above, we get:
  *
  *           C=(0.10211919, -0.3926596 , -1.16225222)
  *           Rc =( [0.9916659 ,  0.03033392,  0.12521416],
  *                 [-0.05255451, 0.98259354,  0.17817956],
  *                 [-0.11762974, -0.18327516,  0.9759987])
  *
  *           for the data set(bottom right is world origin) above, we get:
  *
  *           C=(0.057647444,0.51347721,-1.1624194)
  *           Rc=(  [-0.99165297, -0.028165514, -0.12582189],
                    [0.051142283, -0.98172361, -0.18331185],
                    [-0.11835925, -0.18821655, 0.97496957] )


/*
            float Q[4];
                 double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);

                     if (trace > 0.0)
                     {
                         double s = sqrt(trace + 1.0);
                         Q[3] = (s * 0.5);
                         s = 0.5 / s;
                         Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
                         Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
                         Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
                     }

                     else
                     {
                         int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0);
                         int j = (i + 1) % 3;
                         int k = (i + 2) % 3;

                         double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
                         Q[i] = s * 0.5;
                         s = 0.5 / s;

                         Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
                         Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
                         Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
                     }

                   cout <<"the quaternions are :" <<"("<<Q[0]<<","<<Q[1]<<","<<Q[2]<<","<<Q[3]<<")" <<endl;
*/
    imshow("kp",drawkp);



/*
    while (key <= 0) {

        // grab the current RGB matrix
        cv::Mat src = libfreenect2OpenCV.getRGBd();
        cv::Mat img ;
        cvtColor(src,img,CV_BGRA2BGR);
        cv::Mat edges = img;

        // opencv to gray scale
        cv::cvtColor(edges, edges, CV_BGR2GRAY);
        cv::Canny(edges, edges, 120, 175);


        Mat edges_new =edges.clone();
        vector<Vec4i> lines;
        HoughLinesP(edges_new,lines,1,CV_PI /180, 50, 50, 10);
        Mat drawline =Mat::zeros(img.size(),CV_8UC1);

        for(size_t i =0; i<lines.size();i++)
        {
            Vec4i l =lines[i];
            line(drawline,Point(l[0],l[1]),Point(l[2],l[3]),Scalar(255),1,CV_AA);
        }


        for(int i =0; i<edges_new.rows;i++)
        {
            for(int j =0; j <edges_new.cols;j++)
            {
                if((int)drawline.at<uchar>(i,j) !=0)
                {
                    edges_new.at<uchar>(i,j) =0;
                }
            }
        }

      vector<vector<Point> > contours;
      vector<Vec4i> hierarchy;
      findContours(edges_new,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE,Point());

      vector<Rect> boundRect(contours.size());
      vector<float> rectarea(contours.size());

   //   Mat imagecontour =Mat::zeros(edges_new.size(),CV_8UC3);

      for(size_t i =0; i<contours.size();i++)
      {
          boundRect[i] =boundingRect(Mat(contours[i]));
          rectarea[i]=boundRect[i].width*boundRect[i].height;
          if(rectarea[i]>350)
          {
              rectangle(img,boundRect[i].tl(), boundRect[i].br(),Scalar(0,255,0),1,8,0);
          }
      }


        // Show canny edges
        cv::imshow("Canny Edges", edges);

        cv::namedWindow("rgbd",WINDOW_NORMAL);
        cv::resizeWindow("rgbd",1920,1080);
        cv::imshow("rgbd",img);

        // Show original RGB Image
       // cv::imshow("RGB Matrix", img);

      //  cv::imshow("imagecontour",imagecontour);

        // Show depth matrix
     //   cv::imshow("Depth Matrix", libfreenect2OpenCV.getDepthMatUndistorted());

        // Show depth matrix
     //   cv::imshow("IR Matrix", libfreenect2OpenCV.getIRMat());
     usleep(500000);
        key = cv::waitKey(1);
    }
*/
    cv::waitKey(0);
    libfreenect2OpenCV.stop();
}

