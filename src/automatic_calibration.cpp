//subscribe to rgb input and look in the incoming stream for a a certain object (baxter gripper)
#include <ros/ros.h>
#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>


using namespace std;
using namespace cv;

//global variables declaration
Mat Ipics,greyIpics, pics,greypics,DescriptorI,DescriptorC,KeyIpics,KeyCpics,Matching, mean_;
SurfFeatureDetector Detector(400);
SurfDescriptorExtractor Extractor;
FlannBasedMatcher Matcher;
vector<KeyPoint> KeypointsI, KeypointsC,GoodKeypointsI;

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    namedWindow("GreyPics",CV_WINDOW_AUTOSIZE);
    namedWindow("GreyKeyPointPics",CV_WINDOW_AUTOSIZE);
    namedWindow("ShowMatch",CV_WINDOW_AUTOSIZE);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
    }
    //get current picture
    pics = cv_ptr->image;

    //convert it to greyscale
    cvtColor(pics,greypics,CV_BGR2GRAY);

    //get its keypoints
    Detector.detect(greypics,KeypointsC);

    //get descriptors
    Extractor.compute(greypics, KeypointsC,DescriptorC);

    //show it
    drawKeypoints(greypics,KeypointsC,KeyCpics);

    imshow("GreyKeyPointPics",KeyCpics);

    //match them
    vector<DMatch> OutMatch,GoodOutMatch;
    Matcher.match(DescriptorI,DescriptorC,OutMatch);


    double max_dist = 0; double min_dist = 100;

    for( int i = 0; i < DescriptorI.rows; i++ )
    { double dist = OutMatch[i].distance;
      if(dist<min_dist) min_dist = dist;
      if(dist>max_dist) max_dist = dist;
    }

    for( int i = 0; i < DescriptorI.rows; i++ )
    { if( OutMatch[i].distance < 0.2 )
      { GoodOutMatch.push_back(OutMatch[i]);}
    }

    vector<Point2f> VI, VC ;
    for (unsigned int i=0;i<GoodOutMatch.size();i++)
    {
      VI.push_back(KeypointsI[GoodOutMatch[i].queryIdx].pt);
      VC.push_back(KeypointsC[GoodOutMatch[i].trainIdx].pt);
    }
    drawMatches(greyIpics,KeypointsI,greypics,KeypointsC,GoodOutMatch,Matching);

    reduce(VC,mean_,0,CV_REDUCE_AVG);
    Point2f mean(mean_.at<float>(0,0), mean_.at<float>(0,1));
    circle(greypics, cv::Point(mean.x, mean.y), 10, CV_RGB(255,0,0));

    /*
    perspectiveTransform(obj_corners, scene_corners, TMatrix);

    line(Matching, scene_corners[0] + Point2f(greyIpics.cols, 0),
          scene_corners[1] + Point2f(greyIpics.cols, 0),
          Scalar(0, 255, 0), 4);
    line(Matching, scene_corners[1] + Point2f(greyIpics.cols, 0),
          scene_corners[2] + Point2f(greyIpics.cols, 0),
          Scalar(0, 255, 0), 4);
    line(Matching, scene_corners[2] + Point2f(greyIpics.cols, 0),
          scene_corners[3] + Point2f(greyIpics.cols, 0),
          Scalar(0, 255, 0), 4);
    line(Matching, scene_corners[3] + Point2f(greyIpics.cols, 0),
          scene_corners[0] + Point2f(greyIpics.cols, 0),
          Scalar(0, 255, 0), 4);*/


    imshow("GreyPics",greypics);
    imshow("ShowMatch", Matching);


    waitKey(1);

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "automatic_calibration");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);
    image_transport::Subscriber in_image = it_.subscribe("/camera/rgb/image_raw",1,imageCb);

    //get the picture of the reference that we will look for later
    Ipics = imread("/home/ghanim/git/catkin_ws/baxter_gripper.jpg");
    //convert to greyscale
    cvtColor(Ipics,greyIpics,CV_BGR2GRAY);

    //get keypoints
    Detector.detect(greyIpics,KeypointsI);

    Extractor.compute(greyIpics, KeypointsI,DescriptorI);

    drawKeypoints(greyIpics,KeypointsI,KeyIpics);

    namedWindow("OriginalPic",CV_WINDOW_AUTOSIZE);
    namedWindow("GreyBasePics",CV_WINDOW_AUTOSIZE);
    namedWindow("GreyBaseKeyPointPics",CV_WINDOW_AUTOSIZE);

    imshow("OriginalPic",Ipics);
    imshow("GreyBasePics",greyIpics);
    imshow("GreyBaseKeyPointPics",KeyIpics);


    ros::spin();
    cout << " ************* I finished ******************** " << endl;
    return 0;
}

//Good
/*#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/features2d.hpp>

using namespace std;
using namespace cv;

int main()
{

    Mat Ipics,greyIpics, pics,greypics,DescriptorI,DescriptorC,KeyIpics,KeyCpics,Matching;

    SurfFeatureDetector Detector(400);
    SurfDescriptorExtractor Extractor;
    FlannBasedMatcher Matcher;

    vector<KeyPoint> KeypointsI, KeypointsC,GoodKeypointsI;
    vector<Point2f> obj_corners(4),scene_corners(4);

    obj_corners[0] = cvPoint(24,44);
    obj_corners[1] = cvPoint(196,44);
    obj_corners[3] = cvPoint(24,210);
    obj_corners[2] = cvPoint(196,210);

    scene_corners[0] = cvPoint(24,44);
    scene_corners[1] = cvPoint(196,44);
    scene_corners[3] = cvPoint(24,210);
    scene_corners[2] = cvPoint(196,210);

    VideoCapture cap("/home/ghanim/git/catkin_ws/video2.mp4");

    cap.read(Ipics);
    cvtColor(Ipics,greyIpics,CV_BGR2GRAY);
    Detector.detect(greyIpics,KeypointsI);
    for (unsigned int i=0;i<KeypointsI.size();i++)
    {
        if ((KeypointsI[i].pt.x > obj_corners[0].x)
                & (KeypointsI[i].pt.x < obj_corners[1].x)
                & (KeypointsI[i].pt.y > obj_corners[0].y)
                & (KeypointsI[i].pt.y < obj_corners[3].y))
            GoodKeypointsI.push_back(KeypointsI[i]);
    }

    Extractor.compute(greyIpics, GoodKeypointsI,DescriptorI);

    drawKeypoints(greyIpics,GoodKeypointsI,KeyIpics);

    namedWindow("GreyBasePics",CV_WINDOW_AUTOSIZE);
    namedWindow("GreyBaseKeyPointPics",CV_WINDOW_AUTOSIZE);
    namedWindow("GreyPics",CV_WINDOW_AUTOSIZE);
    namedWindow("GreyKeyPointPics",CV_WINDOW_AUTOSIZE);
    namedWindow("ShowMatch",CV_WINDOW_AUTOSIZE);

    imshow("GreyBasePics",greyIpics);
    imshow("GreyBaseKeyPointPics",KeyIpics);

    while(cap.read(pics))
    {
        cv::cvtColor(pics,greypics,CV_BGR2GRAY);
        Detector.detect(greypics,KeypointsC);

        vector<KeyPoint> GoodKeypointsC;

        float xmin=1000;
        float xmax=0;
        float ymin=1000;
        float ymax=0;

        for (unsigned int i=0;i<scene_corners.size();i++)
        {
            if (scene_corners[i].x < xmin)
                xmin=scene_corners[i].x;
            if (scene_corners[i].x > xmax)
                xmax=scene_corners[i].x;
            if (scene_corners[i].y < ymin)
                ymin=scene_corners[i].y;
            if (scene_corners[i].y > ymax)
                ymax=scene_corners[i].y;
        }

        for (unsigned int i=0;i<KeypointsC.size();i++)
            {
                if ((KeypointsC[i].pt.x > xmin)
                        & (KeypointsC[i].pt.x < xmax)
                        & (KeypointsC[i].pt.y > ymin)
                        & (KeypointsC[i].pt.y < ymax))
                    GoodKeypointsC.push_back(KeypointsC[i]);
            }
        Extractor.compute(greypics, GoodKeypointsC,DescriptorC);

        drawKeypoints(greypics,GoodKeypointsC,KeyCpics);

        imshow("GreyPics",greypics);
        imshow("GreyKeyPointPics",KeyCpics);

        vector<DMatch> OutMatch,GoodOutMatch;
        Matcher.match(DescriptorI,DescriptorC,OutMatch);


        double max_dist = 0; double min_dist = 100;

        for( int i = 0; i < DescriptorI.rows; i++ )
        { double dist = OutMatch[i].distance;
            if(dist<min_dist) min_dist = dist;
            if(dist>max_dist) max_dist = dist;
        }

        for( int i = 0; i < DescriptorI.rows; i++ )
        { if( OutMatch[i].distance < 0.2 )
            { GoodOutMatch.push_back(OutMatch[i]);}
        }

        vector<Point2f> VI, VC ;
        for (unsigned int i=0;i<GoodOutMatch.size();i++)
        {
            VI.push_back(GoodKeypointsI[GoodOutMatch[i].queryIdx].pt);
            VC.push_back(GoodKeypointsC[GoodOutMatch[i].trainIdx].pt);
        }
        drawMatches(greyIpics,GoodKeypointsI,greypics,GoodKeypointsC,GoodOutMatch,Matching);
        Mat TMatrix=findHomography(VI,VC,CV_RANSAC);


        perspectiveTransform(obj_corners, scene_corners, TMatrix);

        line(Matching, scene_corners[0] + Point2f(greyIpics.cols, 0),
                scene_corners[1] + Point2f(greyIpics.cols, 0),
                Scalar(0, 255, 0), 4);
        line(Matching, scene_corners[1] + Point2f(greyIpics.cols, 0),
                scene_corners[2] + Point2f(greyIpics.cols, 0),
                Scalar(0, 255, 0), 4);
        line(Matching, scene_corners[2] + Point2f(greyIpics.cols, 0),
                scene_corners[3] + Point2f(greyIpics.cols, 0),
                Scalar(0, 255, 0), 4);
        line(Matching, scene_corners[3] + Point2f(greyIpics.cols, 0),
                scene_corners[0] + Point2f(greyIpics.cols, 0),
                Scalar(0, 255, 0), 4);



        imshow("ShowMatch", Matching);


        waitKey(1);
    }
    cout << " ************* I finished ******************** " << endl;
    return 0;
}*/
