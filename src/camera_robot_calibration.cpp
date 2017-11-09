#include "camera_robot_calibration.hpp"

using namespace camera_robot_calibration;

void CALIBRATOR::init(){
    ROS_INFO("CALIBRITOR: initializing ......");
    _nh_clibrator.getParam("/calibration_parameters", _global_parameters.get_parameters());
    _global_parameters.set_marker_size(std::stof(_global_parameters.get_parameters()["marker_size"]));
    _global_parameters.set_epsilon(std::stof(_global_parameters.get_parameters()["epsilon"]));
    _global_parameters.set_first_iteration(true);

    //set the robot
    std::string robot_name = std::string(_global_parameters.get_parameters()["robot"]);
    std::string camera_name = std::string(_global_parameters.get_parameters()["camera"]);
    if(strcmp(robot_name.c_str(), "baxter") == 0){
        _robot.reset(new BAXTER);
        _robot->init();
    }
    else if(strcmp(robot_name.c_str(), "crustcrawler") == 0){
        _robot.reset(new CRUSTCRAWLER);
        _robot->init();
    }

    //set the camera
    if(strcmp(camera_name.c_str(), "kinect_v2") == 0){
        _camera.reset(new CAMERA_kinect_v2);
        _camera->init();
    }
    else if(strcmp(camera_name.c_str(), "kinect_freenect") == 0){
        _camera.reset(new CAMERA_kinect_freenect);
        _camera->init();
    }
    else if(strcmp(camera_name.c_str(), "kinect_openni") == 0){
        _camera.reset(new CAMERA_kinect_openni);
        _camera->init();
    }

    else if(strcmp(camera_name.c_str(), "optitrack") == 0){
        _camera.reset(new CAMERA_optitrack);
        _camera->init();
    }

    //set number of points to calibrate
    int number_points = std::stoi(_global_parameters.get_parameters()["number_of_points"]);
    if(number_points != _global_parameters.get_number_of_points())
        _global_parameters.set_number_of_points(number_points);
    _global_parameters.get_markers_positions_camera_frame().resize(_global_parameters.get_number_of_points(), 4);
    _global_parameters.get_markers_positions_robot_frame().resize(_global_parameters.get_number_of_points(), 4);
    _calibrator_spinner.reset(new ros::AsyncSpinner(1));
    _calibrator_spinner->start();
    ROS_INFO("CALIBRITOR: initialized");
}


void CALIBRATOR::acquire_optitrack_points(){
    _global_parameters.set_camera_topics_status(false);
    usleep(1e6);
    Eigen::Vector3d marker_robot_frame = _global_parameters.get_robot_eef_position();
    Eigen::Vector3d marker_optitrack_frame;
    if(!_global_parameters.get_camera_topics_status()){
        ROS_WARN("CALIBRITOR: Make sure that the markers are in optitrack field of view");
        return;
    }
    marker_optitrack_frame << _global_parameters.get_optitrack_marker_msg()->pose.position.x,
            _global_parameters.get_optitrack_marker_msg()->pose.position.y,
            _global_parameters.get_optitrack_marker_msg()->pose.position.z;
    _global_parameters.get_markers_positions_camera_frame().row(_global_parameters.get_number_of_validated_points())
            << marker_optitrack_frame(0), marker_optitrack_frame(1), marker_optitrack_frame(2), 1;

    _global_parameters.get_markers_positions_robot_frame().row(_global_parameters.get_number_of_validated_points())
            << marker_robot_frame(0), marker_robot_frame(1), marker_robot_frame(2), 1.0;
    int iteration_number = _global_parameters.get_number_of_validated_points()++;
    ROS_INFO_STREAM("CALIBRATOR: marker position is: " << marker_optitrack_frame);
    ROS_WARN_STREAM("CALIBRATOR: the supposed number is: " << iteration_number);

}

void CALIBRATOR::show_image(){
    try{
        cv::namedWindow("ShowMarker",CV_WINDOW_AUTOSIZE);
        cv::imshow("ShowMarker", _global_parameters.get_raw_original_picture());
        cv::waitKey(1);
    }
    catch(...)
    {
        ROS_ERROR("Something went wrong !!!");
        return;
    }

}

void CALIBRATOR::acquire_points(){
    ROS_INFO("CALIBRATOR: at acquiring points");
    //while(!_global_parameters.get_camera_topics_status());
    _global_parameters.get_rgb_msg().reset(new sensor_msgs::Image(_camera->_syncronized_camera_sub->get_rgb()));
    _global_parameters.get_depth_msg().reset(new sensor_msgs::Image(_camera->_syncronized_camera_sub->get_depth()));
    _global_parameters.get_camera_info_msg().reset(new sensor_msgs::CameraInfo(_camera->_syncronized_camera_sub->get_rgb_info()));
    Eigen::Vector3d marker_robot_frame = _global_parameters.get_robot_eef_position();
    std::vector<double> current_eef_position = {marker_robot_frame(0), marker_robot_frame(1), marker_robot_frame(2)};
    if(_global_parameters.get_first_iteration()){
        _global_parameters.set_last_eef_position(current_eef_position);
        _global_parameters.set_first_iteration(false);
    }

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
                if(pt_marker.data[0] != pt_marker.data[0] &&
                        pt_marker.data[1] != pt_marker.data[1] &&
                        pt_marker.data[2] != pt_marker.data[2]){
                    ROS_WARN("CALIBRATOR: point is NAN");
                    return;
                }
                else{
                    int iteration_number = -1;
                    double difference = calibration_helpers_methods::largest_difference(current_eef_position, _global_parameters.get_last_eef_position());
                    ROS_WARN_STREAM("CALIBRATOR: the difference is: " << difference);
                    if(difference > _global_parameters.get_epsilon()){
                        //save points only if end effector position changes with an epsilon
                        _global_parameters.get_markers_positions_camera_frame().row(_global_parameters.get_number_of_validated_points())
                                << pt_marker.data[0], pt_marker.data[1], pt_marker.data[2] , 1.0;

                        _global_parameters.get_markers_positions_robot_frame().row(_global_parameters.get_number_of_validated_points())
                                << marker_robot_frame(0), marker_robot_frame(1), marker_robot_frame(2), 1.0;
                        iteration_number = _global_parameters.get_number_of_validated_points()++;
                        ROS_WARN_STREAM("CALIBRATOR: the supposed number is: " << iteration_number);
                    }
                    else
                        ROS_WARN_STREAM("CALIBRATOR: Failed iteration: " << iteration_number);
                }
                _global_parameters.set_last_eef_position(current_eef_position);
            }
        }
        //        ROS_WARN_STREAM("CALIBRATOR: the supposed number is: " << _global_parameters.get_number_of_validated_points()++);
    }
    catch(...)
    {
        ROS_ERROR("Something went wrong !!!");
        return;
    }
}

bool CALIBRATOR::solve_for_transformation_matrix(){
    Eigen::MatrixXd trans_matrix(4,4);
    Eigen::MatrixX4d test(_global_parameters.get_number_of_points(), 4);
    test.setZero(_global_parameters.get_number_of_points(), 4);
    Eigen::Vector4d v1,v2,v3;

    ROS_WARN("CALIBRATOR: Trying to solve ....");
    if(!_global_parameters.get_markers_positions_camera_frame().isApprox(test)){
        ROS_WARN_STREAM(_global_parameters.get_markers_positions_camera_frame());
        ROS_WARN_STREAM(_global_parameters.get_markers_positions_robot_frame());

        v1 = _global_parameters.get_markers_positions_camera_frame().jacobiSvd(
                    Eigen::ComputeThinU | Eigen::ComputeThinV).solve(_global_parameters.get_markers_positions_robot_frame().col(0));
        v2 = _global_parameters.get_markers_positions_camera_frame().jacobiSvd(
                    Eigen::ComputeThinU | Eigen::ComputeThinV).solve(_global_parameters.get_markers_positions_robot_frame().col(1));
        v3 = _global_parameters.get_markers_positions_camera_frame().jacobiSvd(
                    Eigen::ComputeThinU | Eigen::ComputeThinV).solve(_global_parameters.get_markers_positions_robot_frame().col(2));

        ROS_WARN("CALIBRATOR: solving ....");
        trans_matrix << v1(0), v1(1), v1(2), v1(3),
                v2(0), v2(1), v2(2), v2(3),
                v3(0), v3(1), v3(2), v3(3),
                0,     0,     0,     1;
        Eigen::Vector3d position;
        position << v1(3), v2(3), v3(3);
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
        _global_parameters.set_transformation_position(position);
        return true;
    }
    else
        return false;
}

void CAMERA_optitrack::init(){
    ROS_INFO_STREAM( _nh_camera.getNamespace());
    _camera_topics_sub = _nh_camera.subscribe<geometry_msgs::PoseStamped>("/robot/calibrator/pose", 1, &CAMERA_optitrack::save_markers_positions_cb, this);

    _camera_spinner.reset(new ros::AsyncSpinner(1));
    _camera_spinner->start();
}

void CAMERA_kinect_v2::init(){
    ROS_INFO_STREAM( _nh_camera.getNamespace());
    _syncronized_camera_sub.reset(new rgbd_utils::RGBD_Subscriber("/kinect2/qhd/camera_info",
                                                                  "/kinect2/qhd/image_color_rect",
                                                                  "/kinect2/qhd/camera_info",
                                                                  "/kinect2/qhd/image_depth_rect",
                                                                  _nh_camera));
    _camera_topics_sub = _nh_camera.subscribe<sensor_msgs::Image>("/kinect2/qhd/image_depth_rect", 1, &CAMERA_kinect_v2::camera_topics_start_publishing_cb, this);
    std::string camera_file_path;
    _nh_camera.getParam("/camera_file_path", camera_file_path);
    _global_parameters.get_camera_character().readFromXMLFile(camera_file_path);
    _camera_spinner.reset(new ros::AsyncSpinner(1));
    _camera_spinner->start();
}

void CAMERA_kinect_freenect::init(){
    _syncronized_camera_sub.reset(new rgbd_utils::RGBD_Subscriber("/camera/rgb/camera_info",
                                                                  "/camera/rgb/image_raw",
                                                                  "/camera/depth/camera_info",
                                                                  "/camera/depth/image_raw",
                                                                  _nh_camera));
    _camera_topics_sub = _nh_camera.subscribe<sensor_msgs::Image>("/camera/depth/image_raw", 1, &CAMERA_kinect_freenect::camera_topics_start_publishing_cb, this);
    std::string camera_file_path;
    _nh_camera.getParam("/camera_file_path", camera_file_path);
    _global_parameters.get_camera_character().readFromXMLFile(camera_file_path);
    _camera_spinner.reset(new ros::AsyncSpinner(1));
    _camera_spinner->start();
}

void CAMERA_kinect_openni::init(){
    ROS_INFO_STREAM( _nh_camera.getNamespace());
    _syncronized_camera_sub.reset(new rgbd_utils::RGBD_Subscriber("/camera/rgb/camera_info",
                                                                  "/camera/rgb/image_raw",
                                                                  "/camera/depth_registered/camera_info",
                                                                  "/camera/depth_registered/image_raw",
                                                                  _nh_camera));
    _camera_topics_sub = _nh_camera.subscribe<sensor_msgs::Image>("/camera/depth_registered/image_raw", 1, &CAMERA_kinect_openni::camera_topics_start_publishing_cb, this);
    std::string camera_file_path;
    _nh_camera.getParam("/camera_file_path", camera_file_path);
    _global_parameters.get_camera_character().readFromXMLFile(camera_file_path);
    _camera_spinner.reset(new ros::AsyncSpinner(1));
    _camera_spinner->start();
}

void BAXTER::init(){
    std::string baxter_arm = std::string(_global_parameters.get_parameters()["baxter_arm"]);
    if(strcmp(baxter_arm.c_str(), "left") == 0)
        _eef_state_sub.reset(new ros::Subscriber(_baxter_nh.subscribe("/robot/limb/left/endpoint_state", 10, &BAXTER::eef_Callback, this)));
    else if(strcmp(baxter_arm.c_str(), "right") == 0)
        _eef_state_sub.reset(new ros::Subscriber(_baxter_nh.subscribe("/robot/limb/right/endpoint_state", 10, &BAXTER::eef_Callback, this)));
    _baxter_spinner.reset(new ros::AsyncSpinner(1));
    _baxter_spinner->start();
}

void CRUSTCRAWLER::init(){
    _eef_state_sub.reset(new ros::Subscriber(_crustcrawler_nh.subscribe("/crustcrawler/endpoint_state", 10, &CRUSTCRAWLER::eef_Callback, this)));
    _crustcrawler_spinner.reset(new ros::AsyncSpinner(1));
    _crustcrawler_spinner->start();
}
