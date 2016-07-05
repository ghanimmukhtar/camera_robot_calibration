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

IplImage* img;
IplImage* imgGrayScale;
CvMemStorage *storage = cvCreateMemStorage(0); //storage area for all contours

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
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

    IplImage pic = cv_ptr->image;
    img =  &pic;

     //show the original image
     cvNamedWindow("Raw");
     cvShowImage("Raw",img);

      //converting the original image into grayscale
     imgGrayScale = cvCreateImage(cvGetSize(img), 8, 1);
     cvCvtColor(img,imgGrayScale,CV_BGR2GRAY);

      //thresholding the grayscale image to get better results
     cvThreshold(imgGrayScale,imgGrayScale,128,255,CV_THRESH_BINARY);
     //cvThreshold(imgGrayScale,imgGrayScale,128,255,CV_THRESH_TRUNC);

     CvSeq* contours;  //hold the pointer to a contour in the memory block
     CvSeq* result;   //hold sequence of points of a contour


     //finding all contours in the image
     cvFindContours(imgGrayScale, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));

     //iterating through each contour
     while(contours)
     {
         //obtain a sequence of points of contour, pointed by the variable 'contour'
         result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours)*0.02, 0);

         //if there are 3  vertices  in the contour(It should be a triangle)
        if(result->total==3 )
         {
             //iterating through each point
             CvPoint *pt[3];
             for(int i=0;i<3;i++){
                 pt[i] = (CvPoint*)cvGetSeqElem(result, i);
             }

             //drawing lines around the triangle
             cvLine(img, *pt[0], *pt[1], cvScalar(255,0,0),4);
             cvLine(img, *pt[1], *pt[2], cvScalar(255,0,0),4);
             cvLine(img, *pt[2], *pt[0], cvScalar(255,0,0),4);

         }

          //if there are 4 vertices in the contour(It should be a quadrilateral)
         else if(result->total==4 )
         {
             //iterating through each point
             CvPoint *pt[4];
             for(int i=0;i<4;i++){
                 pt[i] = (CvPoint*)cvGetSeqElem(result, i);
             }

             //drawing lines around the quadrilateral
             cvLine(img, *pt[0], *pt[1], cvScalar(0,255,0),4);
             cvLine(img, *pt[1], *pt[2], cvScalar(0,255,0),4);
             cvLine(img, *pt[2], *pt[3], cvScalar(0,255,0),4);
             cvLine(img, *pt[3], *pt[0], cvScalar(0,255,0),4);
         }

       //if there are 7  vertices  in the contour(It should be a heptagon)
         else if(result->total ==7  )
         {
             //iterating through each point
             CvPoint *pt[7];
             for(int i=0;i<7;i++){
                 pt[i] = (CvPoint*)cvGetSeqElem(result, i);
             }

             //drawing lines around the heptagon
             cvLine(img, *pt[0], *pt[1], cvScalar(0,0,255),4);
             cvLine(img, *pt[1], *pt[2], cvScalar(0,0,255),4);
             cvLine(img, *pt[2], *pt[3], cvScalar(0,0,255),4);
             cvLine(img, *pt[3], *pt[4], cvScalar(0,0,255),4);
             cvLine(img, *pt[4], *pt[5], cvScalar(0,0,255),4);
             cvLine(img, *pt[5], *pt[6], cvScalar(0,0,255),4);
             cvLine(img, *pt[6], *pt[0], cvScalar(0,0,255),4);
         }

          //obtain the next contour
         contours = contours->h_next;
     }

      //show the image in which identified shapes are marked
     cvNamedWindow("Tracked");
     cvShowImage("Tracked",img);

     cvWaitKey(1); //wait for a key press


}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "contour_detection");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);
    image_transport::Subscriber in_image = it_.subscribe("/camera/rgb/image_raw",1,imageCb);
    ros::spin();
    //cleaning up
   cvDestroyAllWindows();
   cvReleaseMemStorage(&storage);
   cout << " ************* Release storage ******************** " << endl;
   cvReleaseImage(&img);
   cout << " ************* Release image ******************** " << endl;
   cvReleaseImage(&imgGrayScale);
    cout << " ************* I finished ******************** " << endl;
    return 0;
}
