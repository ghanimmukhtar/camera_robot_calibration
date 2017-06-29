#include <camera_robot_calibration/camera_robot_calibration.hpp>

using namespace camera_robot_calibration;

void CALIBRATOR::init(){
    ROS_INFO("CALIBRITOR: initializing ......");
    _nh_clibrator.getParam("/calibration_parameters", _global_parameters.get_parameters());
    _global_parameters.set_marker_size(std::stof(_global_parameters.get_parameters()["marker_size"]));

    //set the robot
    std::string robot_name = std::string(_global_parameters.get_parameters()["robot"]);
    std::string camera_name = std::string(_global_parameters.get_parameters()["camera"]);
    if(strcmp(robot_name.c_str(), "baxter") == 0){
        ROS_INFO("CALIBRITOR: initializing robot baxter ......");
        _robot->init();
        ROS_INFO("CALIBRITOR: initializing robot baxter finished.");
    }
    else if(strcmp(robot_name.c_str(), "crustcrawler") == 0){
        ROS_INFO("CALIBRITOR: initializing robot crustcrawler ......");
        CRUSTCRAWLER::Ptr robot;

        robot->init();
        //_robot = robot;
        ROS_INFO("CALIBRITOR: initializing robot crustcrawler finished.");
    }

    //set the camera
    if(strcmp(camera_name.c_str(), "kinect_v2") == 0){
        ROS_INFO("CALIBRITOR: initializing kinect v2......");
        _camera->init();
        ROS_INFO("CALIBRITOR: initializing kinect v2 finished.");
    }
    else if(strcmp(camera_name.c_str(), "kinect_freenect") == 0){
        ROS_INFO("CALIBRITOR: initializing kinect freenect camera ......");
        _camera.reset(new CAMERA_kinect_freenect);
        _camera->init();
        ROS_INFO("CALIBRITOR: initializing kinect freenect finished.");
    }
    else if(strcmp(camera_name.c_str(), "kinect_openni") == 0){
        _camera->init();
    }

    //set number of points to calibrate
    if(std::stoi(_global_parameters.get_parameters()["number_of_points"]) != _global_parameters.get_number_of_points())
        _global_parameters.set_number_of_points(std::stoi(_global_parameters.get_parameters()["number_of_points"]));

    ROS_WARN("CALIBRITOR: Number of point is set");
    _global_parameters.get_markers_positions_camera_frame().resize(_global_parameters.get_number_of_points(), 4);
    _global_parameters.get_markers_positions_robot_frame().resize(_global_parameters.get_number_of_points(), 4);

    ROS_WARN("CALIBRITOR: Matrices are set");
    _calibrator_spinner.reset(new ros::AsyncSpinner(1));
    _calibrator_spinner->start();
}

void CALIBRATOR::acquire_points(){
    ROS_INFO("CALIBRATOR: at acquiring points");
    while(!_global_parameters.get_camera_topics_status());
    _global_parameters.get_rgb_msg().reset(new sensor_msgs::Image(_camera->_syncronized_camera_sub->get_rgb()));
    _global_parameters.get_depth_msg().reset(new sensor_msgs::Image(_camera->_syncronized_camera_sub->get_depth()));
    _global_parameters.get_camera_info_msg().reset(new sensor_msgs::CameraInfo(_camera->_syncronized_camera_sub->get_rgb_info()));
    Eigen::Vector3d marker_robot_frame = _global_parameters.get_robot_eef_position();

    ROS_INFO("CALIBRATOR: topics are set");
    cv::namedWindow("ShowMarker",CV_WINDOW_AUTOSIZE);

    cv_bridge::CvImagePtr cv_ptr = _global_parameters.get_cvt();
    try
    {
        cv_ptr = cv_bridge::toCvCopy(_global_parameters.get_rgb_msg(), sensor_msgs::image_encodings::BGR8);
        _global_parameters.set_raw_original_picture(cv_ptr->image);
        _global_parameters.get_aruco_marker_detector().setDictionary("ARUCO");
        _global_parameters.get_aruco_marker_detector().detect(_global_parameters.get_raw_original_picture(),
                                                              _global_parameters.get_markers(),
                                                              _global_parameters.get_camera_character(),
                                                              0.1);
        _global_parameters.get_marker_centers().resize(_global_parameters.get_markers().size());
        for(size_t i = 0; i < _global_parameters.get_markers().size(); i++){

            _global_parameters.get_markers()[i].draw(_global_parameters.get_raw_original_picture(), cv::Scalar(94.0, 206.0, 165.0, 0.0));

            _global_parameters.get_markers()[i].calculateExtrinsics(_global_parameters.get_marker_size(),
                                                                    _global_parameters.get_camera_character(),
                                                                    false);

            _global_parameters.get_marker_centers()[i] << (int) (_global_parameters.get_markers()[i][0].x + _global_parameters.get_markers()[i][2].x)/2,
                    (int) (_global_parameters.get_markers()[i][0].y + _global_parameters.get_markers()[i][2].y)/2;

            cv::circle(_global_parameters.get_raw_original_picture(), cv::Point(_global_parameters.get_marker_centers()[i](0),
                                                                                _global_parameters.get_marker_centers()[i](1)),
                                                                                10,
                                                                                CV_RGB(255, 0, 0));

                       _global_parameters.set_rgb_cloud_converter(_global_parameters.get_depth_msg(),
                                                                  _global_parameters.get_rgb_msg(),
                                                                  _global_parameters.get_camera_info_msg());
                    _global_parameters.set_pointcloud_msg(_global_parameters.get_rgb_cloud_converter().get_pointcloud());

            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
                    input_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::fromROSMsg(_global_parameters.get_pointcloud_msg(), *input_cloud);
            for(size_t q = 0; q < _global_parameters.get_marker_centers().size(); q++){
                double x = _global_parameters.get_marker_centers()[q](0), y = _global_parameters.get_marker_centers()[q](1);
                pcl::PointXYZRGBA pt_marker = input_cloud->at(std::round(x) + std::round(y) * input_cloud->width);
                if(pt_marker.x == pt_marker.x && pt_marker.y == pt_marker.y && pt_marker.z == pt_marker.z)
                    _global_parameters.get_markers_positions_camera_frame() << pt_marker.x, pt_marker.y, pt_marker.z , 1.0;
            }
            _global_parameters.get_markers_positions_robot_frame() << marker_robot_frame(0), marker_robot_frame(1),
                    marker_robot_frame(2), 1.0;
            _global_parameters.get_number_of_validated_points()++;
            cv::imshow("ShowMarker", _global_parameters.get_raw_original_picture());
            cv::waitKey(1);
        }

    }
    catch(...)
    {
        ROS_ERROR("Something went wrong !!!");
        return;
    }
}

void CALIBRATOR::solve_for_transformation_matrix(){
    Eigen::MatrixXd trans_matrix(4,4);
    Eigen::Vector4d v1,v2,v3;

    v1 = _global_parameters.get_markers_positions_camera_frame().jacobiSvd(
                Eigen::ComputeThinU | Eigen::ComputeThinV).solve(_global_parameters.get_markers_positions_robot_frame().col(0));
    v2 = _global_parameters.get_markers_positions_camera_frame().jacobiSvd(
                Eigen::ComputeThinU | Eigen::ComputeThinV).solve(_global_parameters.get_markers_positions_robot_frame().col(1));
    v3 = _global_parameters.get_markers_positions_camera_frame().jacobiSvd(
                Eigen::ComputeThinU | Eigen::ComputeThinV).solve(_global_parameters.get_markers_positions_robot_frame().col(2));
    trans_matrix << v1(0), v1(1), v1(2), v1(3),
            v2(0), v2(1), v2(2), v2(3),
            v3(0), v3(1), v3(2), v3(3),
            0,     0,     0,     1;

    //get RPY and quaterion
    tf::Matrix3x3 rotation_matrix(v1(0), v1(1), v1(2),
                                  v2(0), v2(1), v2(2),
                                  v3(0), v3(1), v3(2));

    rotation_matrix.getRPY(_global_parameters.get_transformation_RPY()(0),
                           _global_parameters.get_transformation_RPY()(1),
                           _global_parameters.get_transformation_RPY()(2));
    rotation_matrix.getRotation(_global_parameters.get_transformation_quaternion());
    //set transformation matrix in global parameters
    _global_parameters.set_transformation_matrix(trans_matrix);
}

void CAMERA_kinect_v2::init(){
    _syncronized_camera_sub.reset(new rgbd_utils::RGBD_Subscriber("/kinect2/qhd/camera_info",
                                                                  "/kinect2/qhd/image_color_rect",
                                                                  "/kinect2/qhd/camera_info",
                                                                  "/kinect2/qhd/image_depth_rect",
                                                                  _v2_nh));
    _camera_topics_sub = _v2_nh.subscribe<sensor_msgs::Image>("/kinect2/qhd/image_depth_rect", 1, &CAMERA_kinect_v2::camera_topics_start_publishing_cb, this);
    std::string camera_file_path;
    _v2_nh.getParam("/camera_file_path", camera_file_path);
    _global_parameters.get_camera_character().readFromXMLFile(camera_file_path);
    _v2_spinner.reset(new ros::AsyncSpinner(1));
    _v2_spinner->start();
}

void CAMERA_kinect_freenect::init(){
    ROS_INFO_STREAM("CAMERA_kinect_freenect: initializing, name space is: ");
    ROS_INFO_STREAM( _freenect_nh.getNamespace());
    _syncronized_camera_sub.reset(new rgbd_utils::RGBD_Subscriber("/camera/rgb/camera_info",
                                                                  "/camera/rgb/image_raw",
                                                                  "/camera/depth/camera_info",
                                                                  "/camera/depth/image_raw",
                                                                  _freenect_nh));
    ROS_INFO("CAMERA_kinect_freenect: first topic is okay ...");
    _camera_topics_sub = _freenect_nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 1, &CAMERA_kinect_freenect::camera_topics_start_publishing_cb, this);
    ROS_INFO("CAMERA_kinect_freenect: second topic is okay ...");
    std::string camera_file_path;
    _freenect_nh.getParam("/camera_file_path", camera_file_path);
    _global_parameters.get_camera_character().readFromXMLFile(camera_file_path);
    ROS_INFO("CAMERA_kinect_freenect: camera char is okay ...");
    ROS_INFO("CAMERA_kinect_freenect: initializing kinect freenect camera finished!");
    _freenect_spinner.reset(new ros::AsyncSpinner(1));
    _freenect_spinner->start();
    ROS_INFO("CAMERA_kinect_freenect: initializing finished.");
}

void CAMERA_kinect_openni::init(){
    _syncronized_camera_sub.reset(new rgbd_utils::RGBD_Subscriber("/camera/rgb/camera_info",
                                                                  "/camera/rgb/image_raw",
                                                                  "/camera/depth/camera_info",
                                                                  "/camera/depth_registered/image_raw",
                                                                  _openni_nh));
    _camera_topics_sub = _openni_nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 1, &CAMERA_kinect_openni::camera_topics_start_publishing_cb, this);
    std::string camera_file_path;
    _openni_nh.getParam("/camera_file_path", camera_file_path);
    _global_parameters.get_camera_character().readFromXMLFile(camera_file_path);
    _openni_spinner.reset(new ros::AsyncSpinner(1));
    _openni_spinner->start();
}

void BAXTER::init(){
    ROS_INFO("BAXTER: initializing ......");
    std::string baxter_arm = std::string(_global_parameters.get_parameters()["baxter_arm"]);
    ROS_INFO_STREAM("BAXTER: arm is: " << baxter_arm);
    if(strcmp(baxter_arm.c_str(), "left") == 0)
        _eef_state_sub.reset(new ros::Subscriber(_baxter_nh.subscribe("/robot/limb/left/endpoint_state", 10, &BAXTER::eef_Callback, this)));
    else if(strcmp(baxter_arm.c_str(), "right") == 0)
        _eef_state_sub.reset(new ros::Subscriber(_baxter_nh.subscribe("/robot/limb/right/endpoint_state", 10, &BAXTER::eef_Callback, this)));
    _baxter_spinner.reset(new ros::AsyncSpinner(1));
    _baxter_spinner->start();
    ROS_INFO("BAXTER: initializing finished.");
}

void CRUSTCRAWLER::init(){
    _eef_state_sub.reset(new ros::Subscriber(_crustcrawler_nh.subscribe("/crustcrawler/endpoint_state", 10, &CRUSTCRAWLER::eef_Callback, this)));
    _crustcrawler_spinner.reset(new ros::AsyncSpinner(1));
    _crustcrawler_spinner->start();
}
