//subscribe to rgb input and look in the incoming stream for a a certain object (baxter gripper)
#include <ros/ros.h>
#include <iostream>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <aruco/aruco.h>
#include <aruco/dictionary.h>
#include <aruco/aruco.h>
#include <aruco/cameraparameters.h>

#include <sensor_msgs/image_encodings.h>

#include <image_processing/DescriptorExtraction.h>
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/JointState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>


using namespace std;
using namespace cv;

int no_points = 28;
Mat pic;
sensor_msgs::ImageConstPtr rgb_msg;
sensor_msgs::CameraInfoConstPtr info_msg;
float my_x,my_y;
double xc = 0.0, yc = 0.0, zc = 0.0, sanch_x = 0.796, sanch_y = 0.843, sanch_z = 0.531;
Eigen::MatrixXd points_robot(no_points,4),points_camera(no_points,4);
aruco::CameraParameters camera_char;
vector<aruco::Marker> markers;


Eigen::VectorXd left_arm_joints_position(7), joint_values(7);
std::vector<double> joints_values(7);

void jocommCallback(sensor_msgs::JointState jo_state)
{
    left_arm_joints_position(0) = jo_state.position[4]; left_arm_joints_position(1) = jo_state.position[5]; left_arm_joints_position(2) = jo_state.position[2];
    left_arm_joints_position(3) = jo_state.position[3]; left_arm_joints_position(4) = jo_state.position[6]; left_arm_joints_position(5) = jo_state.position[7];
    left_arm_joints_position(6) = jo_state.position[8];
    for (int i = 0; i < joints_values.size(); i++)
        joints_values[i] = left_arm_joints_position(i);
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    rgb_msg = msg;
    namedWindow("ShowMarker",CV_WINDOW_AUTOSIZE);
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
    pic = cv_ptr->image;

    aruco::MarkerDetector my_detector;
    my_detector.setDictionary("ARUCO");
    my_detector.detect(pic,markers,camera_char,0.04);
    //cout << "is markers empty? " << markers.empty() << endl;
    //cout << "marker is valid: " << markers[0].isValid() << endl;
    if (!markers.empty()){
    markers[0].draw(pic,cv::Scalar(94.0, 206.0, 165.0, 0.0));
        markers[0].calculateExtrinsics(0.04,camera_char,false);
        //my_detector.draw(pic,markers);
        //cout << "markers size is: " << markers.size() << endl;
        //cout << "is markers empty? " << markers.empty() << endl;
        my_x = (int) markers[0][0].x;
        my_y = (int) markers[0][0].y;
        /*cout << "c1_x value in image is: " << (int) markers[0][0].x << endl;
    cout << "c1_y value in image is: " << (int) markers[0][0].y << endl;
        circle(pic, cv::Point(my_x, my_y), 10, CV_RGB(255,0,0));

        cout << "c2_x value in image is: " << (int) markers[0][1].x << endl;
    cout << "c2_y value in image is: " << (int) markers[0][1].y << endl;

        cout << "c3_x value in image is: " << (int) markers[0][2].x << endl;
    cout << "c3_y value in image is: " << (int) markers[0][2].y << endl;

        cout << "c4_x value in image is: " << (int) markers[0][3].x << endl;
    cout << "c4_y value in image is: " << (int) markers[0][3].y << endl;*/
        circle(pic, cv::Point(markers[0][0].x, markers[0][0].y), 10, CV_RGB(255,0,0));
        //cout << "translation vector is: " << markers[0].Tvec << endl;
    }
    imshow("ShowMarker", pic);
    waitKey(1);
}

void infoimageCb(const sensor_msgs::CameraInfoConstPtr msg){
    info_msg = msg;
}

void depthimageCb(const sensor_msgs::ImageConstPtr& depth_msg){
    if(!pic.empty() && !markers.empty()){
        rgbd_utils::RGBD_to_Pointcloud converter(depth_msg, rgb_msg, info_msg);
        sensor_msgs::PointCloud2 ptcl_msg = converter.get_pointcloud();
        image_processing::PointCloudT::Ptr input_cloud(new image_processing::PointCloudT);

        pcl::fromROSMsg(ptcl_msg, *input_cloud);
        image_processing::PointT pt = input_cloud->at((int) my_x+(int) my_y*input_cloud->width);
        zc = pt.z;
        xc = pt.x;
        yc = pt.y;
        //cout << "depth value is: " << pt.z << endl;
        //cout << "x value is: " << (my_x - 319.5)*pt.z/570.3422241210938 << endl;
        //cout << "y value is: " << (my_y - 239.5)*pt.z/570.3422241210938 << endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "baxter_kinect_transformation_mat");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);
    camera_char.readFromXMLFile("/home/mukhtar/git/catkin_ws/src/automatic_camera_robot_cal/data/camera_param_baxter.xml");
    image_transport::Subscriber in_image = it_.subscribe("/camera/rgb/image_rect_color",1,imageCb);
    ros::Subscriber in_info_image = node.subscribe<sensor_msgs::CameraInfoConstPtr>("/camera/rgb/camera_info",1,infoimageCb);
    image_transport::Subscriber in_depth_image = it_.subscribe("/camera/depth_registered/sw_registered/image_rect",1,depthimageCb);
    ros::Subscriber sub_jointmsg;
    sub_jointmsg = node.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
    ros::AsyncSpinner my_spinner(4);
    my_spinner.start();
    Eigen::VectorXd current_position(3);
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    robot_state::RobotState my_robot_state(robot_model);

    std::vector <std::string> variable_names(7);
    variable_names[0] = "left_s0"; variable_names[1] = "left_s1"; variable_names[2] = "left_e0";
    variable_names[3] = "left_e1"; variable_names[4] = "left_w0"; variable_names[5] = "left_w1";
    variable_names[6] = "left_w2";

    std::string input;

    //do the porcess no_points times to save no_points robot left end effector positions
    Eigen::VectorXd my_values;
    int positions = 0;
    while (positions < no_points){
        std::cout << "move the marker to another point and type (next) ..... " << std::endl;
        std::cin.ignore();
        while(xc != xc || yc != yc || zc != zc);
        points_camera(positions,0) = xc; points_camera(positions,1) = yc;
        points_camera(positions,2) = zc; points_camera(positions,3) = 1.0;
        my_robot_state.setVariablePositions(variable_names,joints_values);
        my_robot_state.copyJointGroupPositions(my_robot_state.getRobotModel()->getJointModelGroup("left_arm"),my_values);
        current_position = my_robot_state.getGlobalLinkTransform("left_gripper_base").translation();
        points_robot(positions,0) = current_position(0); points_robot(positions,1) = current_position(1);
        points_robot(positions,2) = current_position(2); points_robot(positions,3) = 1.0;

        /*std::cout << "X is: " << current_position(0) << std::endl
                     << "Y is: " << current_position(1) << std::endl
                        << "Z is: " << current_position(2) << std::endl
                           << "*************************************************" << std::endl;
        //std::cout << my_robot_state.getGlobalLinkTransform("left_gripper").translation() << std::endl;*/
        std::cout << "camera point is: " << std::endl << points_camera.row(positions) << std::endl << "*************************************************" << std::endl;
        std::cout << "robot point is: " << std::endl << points_robot.row(positions) << std::endl << "*************************************************" << std::endl;
        positions += 1;
    }
    std::cout << "camera_points are: " << std::endl
                 << points_camera << std::endl
                    << " ******************************** " << std::endl
                    << "robot_points are: " << std::endl
                                     << points_robot << std::endl
                                        << " ******************************** " << std::endl;
    Eigen::MatrixXd trans_matrix(4,4);
    Eigen::Vector4d v1,v2,v3;

    v1 = points_camera.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(points_robot.col(0));
    v2 = points_camera.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(points_robot.col(1));
    v3 = points_camera.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(points_robot.col(2));
    trans_matrix << v1(0), v1(1), v1(2), v1(3),
                    v2(0), v2(1), v2(2), v2(3),
                    v3(0), v3(1), v3(2), v3(3),
                        0,     0,     0,     1;

    std::cout << "here is the transformation matrix: " << std::endl << trans_matrix << std::endl;
    return 0;
}
