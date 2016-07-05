//subscribe to rgb input and look in the incoming stream for a a certain object (baxter gripper)

#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/image_encodings.h>

#include <image_processing/DescriptorExtraction.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>


using namespace std;
using namespace cv;

//global variables declaration
Mat Ipics,greyIpics, pics,greypics,DescriptorI,DescriptorC,KeyIpics,KeyCpics,Matching, mean_;
SurfFeatureDetector Detector(400);
SurfDescriptorExtractor Extractor;
FlannBasedMatcher Matcher;
vector<KeyPoint> KeypointsI, KeypointsC,GoodKeypointsI,my_keypoints;
cv_bridge::CvImagePtr cv_ptr;
sensor_msgs::ImageConstPtr rgb_msg;
sensor_msgs::CameraInfoConstPtr info_msg;
float my_x,my_y;

void rgbimageCb(const sensor_msgs::ImageConstPtr& msg)
{
    rgb_msg = msg;
    namedWindow("GreyPics",CV_WINDOW_AUTOSIZE);
    namedWindow("GreyKeyPointPics",CV_WINDOW_AUTOSIZE);
    namedWindow("ShowMatch",CV_WINDOW_AUTOSIZE);

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
    Point2f mean(mean_.at<float>(0,0),mean_.at<float>(0,1));
    my_x = mean.x;
    my_y = mean.y;
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

void infoimageCb(const sensor_msgs::CameraInfoConstPtr msg){
    info_msg = msg;
}

void depthimageCb(const sensor_msgs::ImageConstPtr& depth_msg){
    if(!pics.empty()){
        rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, rgb_msg, info_msg);
        sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
        image_processing::PointCloudT::Ptr input_cloud(new image_processing::PointCloudT);

        pcl::fromROSMsg(ptcl_msg, *input_cloud);
        image_processing::PointT pt = input_cloud->at((int) my_x+(int) my_y*input_cloud->width);

        cout << "depth value is: " << pt.z << endl;
        cout << "x value is: " << (my_x - 319.5)*pt.z/570.3422241210938 << endl;
        cout << "y value is: " << (my_y - 239.5)*pt.z/570.3422241210938 << endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "automatic_calibration");
    ros::NodeHandle node;

    image_transport::ImageTransport it_(node);
    image_transport::Subscriber in_rgb_image = it_.subscribe("/camera/rgb/image_raw",1,rgbimageCb);
    ros::Subscriber in_info_image = node.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info",1,infoimageCb);
    image_transport::Subscriber in_depth_image = it_.subscribe("/camera/depth/image_raw",1,depthimageCb);

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
