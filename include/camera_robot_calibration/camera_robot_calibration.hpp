#ifndef _BAXTER_MOVER_HPP
#define _BAXTER_MOVER_HPP

#include "parameters.hpp"
#include "helpers_methods.hpp"

namespace camera_robot_calibration {
Data_config _global_parameters;
class CAMERA_kinect_freenect{
public:
    typedef std::shared_ptr<CAMERA_kinect_freenect> Ptr;
    typedef const std::shared_ptr<CAMERA_kinect_freenect> ConstPtr;

    CAMERA_kinect_freenect(){
    }

    void init();
    void camera_topics_start_publishing_cb(const sensor_msgs::Image::ConstPtr& depth_msgs){
        _global_parameters.set_camera_topics_status(true);
    }
    std::shared_ptr<rgbd_utils::RGBD_Subscriber> _syncronized_camera_sub;
private:
    ros::NodeHandle _freenect_nh;
    std::unique_ptr<ros::AsyncSpinner> _freenect_spinner;
    ros::Subscriber _camera_topics_sub;
};

class CAMERA_kinect_openni{
public:
    typedef std::shared_ptr<CAMERA_kinect_openni> Ptr;
    typedef const std::shared_ptr<CAMERA_kinect_openni> ConstPtr;

    CAMERA_kinect_openni(){
    }

    void init();
    void camera_topics_start_publishing_cb(const sensor_msgs::Image::ConstPtr& depth_msgs){
        _global_parameters.set_camera_topics_status(true);
    }
    std::shared_ptr<rgbd_utils::RGBD_Subscriber> _syncronized_camera_sub;
private:
    ros::NodeHandle _openni_nh;
    std::unique_ptr<ros::AsyncSpinner> _openni_spinner;
    ros::Subscriber _camera_topics_sub;
};

class CAMERA_kinect_v2{
public:
    typedef std::shared_ptr<CAMERA_kinect_v2> Ptr;
    typedef const std::shared_ptr<CAMERA_kinect_v2> ConstPtr;

    CAMERA_kinect_v2(){
    }

    void init();
    void camera_topics_start_publishing_cb(const sensor_msgs::Image::ConstPtr& depth_msgs){
        _global_parameters.set_camera_topics_status(true);
    }
    std::shared_ptr<rgbd_utils::RGBD_Subscriber> _syncronized_camera_sub;
private:
    ros::NodeHandle _v2_nh;
    std::unique_ptr<ros::AsyncSpinner> _v2_spinner;
    ros::Subscriber _camera_topics_sub;
};

class BAXTER{
public:
    typedef std::shared_ptr<BAXTER> Ptr;
    typedef const std::shared_ptr<BAXTER> ConstPtr;

    BAXTER(){}

    void init();

    //call back that register baxter left end effector pose and rearrange the orientation in RPY
    void eef_Callback(const baxter_core_msgs::EndpointState::ConstPtr& eef_feedback){
        calibration_helpers_methods::locate_eef_pose(eef_feedback->pose, _global_parameters);
        //ROS_WARN_STREAM("locating eef stuff gave for position: " << data_simu.get_eef_position()
                    //    << "\n and for orientation: " << data_simu.get_eef_rpy_orientation());
    }
private:
    ros::NodeHandle _baxter_nh;
    std::unique_ptr<ros::AsyncSpinner> _baxter_spinner;
    std::shared_ptr<ros::Subscriber> _eef_state_sub;
};

class CRUSTCRAWLER {
public:
    typedef std::shared_ptr<CRUSTCRAWLER> Ptr;
    typedef const std::shared_ptr<CRUSTCRAWLER> ConstPtr;

    CRUSTCRAWLER(){}
    void init();

    //call back that register baxter left end effector pose and rearrange the orientation in RPY
    void eef_Callback(const crustcrawler_core_msgs::EndpointState::ConstPtr& eef_feedback){
        calibration_helpers_methods::locate_eef_pose(eef_feedback->pose, _global_parameters);
        //ROS_WARN_STREAM("locating eef stuff gave for position: " << data_simu.get_eef_position()
                    //    << "\n and for orientation: " << data_simu.get_eef_rpy_orientation());
    }
private:
    ros::NodeHandle _crustcrawler_nh;
    std::unique_ptr<ros::AsyncSpinner> _crustcrawler_spinner;
    std::shared_ptr<ros::Subscriber> _eef_state_sub;
};

class CALIBRATOR{
public:
    typedef std::shared_ptr<CALIBRATOR> Ptr;
    typedef const std::shared_ptr<CALIBRATOR> ConstPtr;

    CALIBRATOR(){
        init();
    }

    void init();

    void acquire_points();
    void solve_for_transformation_matrix();

    Data_config& get_global_parameters(){
        return _global_parameters;
    }
private:
    ros::NodeHandle _nh_clibrator;
    std::unique_ptr<ros::AsyncSpinner> _calibrator_spinner;
    BAXTER::Ptr _robot;
    CAMERA_kinect_freenect::Ptr _camera;
};

}

#endif //_BAXTER_MOVER_HPP
