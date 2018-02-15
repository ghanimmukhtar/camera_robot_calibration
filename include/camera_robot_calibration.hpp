#ifndef _BAXTER_MOVER_HPP
#define _BAXTER_MOVER_HPP

#include "calibration_helpers_methods.hpp"

namespace camera_robot_calibration {
Calibration_Data_config _global_parameters;
class ROBOT;
class CAMERA;
class CALIBRATOR;

class CAMERA{
public:
    typedef std::shared_ptr<CAMERA> Ptr;
    typedef const std::shared_ptr<CAMERA> ConstPtr;

    CAMERA(){}
    virtual void init() = 0;

    friend class CALIBRATOR;
protected:
    ros::NodeHandle _nh_camera;
    std::unique_ptr<ros::AsyncSpinner> _camera_spinner;
    ros::Subscriber _camera_topics_sub;
    std::shared_ptr<rgbd_utils::RGBD_Subscriber> _syncronized_camera_sub;
};

class CAMERA_optitrack : public CAMERA{
public:
    typedef std::shared_ptr<CAMERA_optitrack> Ptr;
    typedef const std::shared_ptr<CAMERA_optitrack> ConstPtr;

    CAMERA_optitrack(){
    }

    void init() override;
    void save_markers_positions_cb(const geometry_msgs::PoseStamped::ConstPtr& marker_pose){
        _global_parameters.set_camera_topics_status(true);
        calibration_helpers_methods::locate_optitrack_marker_position(marker_pose, _global_parameters);
    }
};

class CAMERA_kinect_freenect : public CAMERA{
public:
    typedef std::shared_ptr<CAMERA_kinect_freenect> Ptr;
    typedef const std::shared_ptr<CAMERA_kinect_freenect> ConstPtr;

    CAMERA_kinect_freenect(){
    }

    void init() override;
    void camera_topics_start_publishing_cb(const sensor_msgs::Image::ConstPtr& depth_msgs){
        _global_parameters.set_camera_topics_status(true);
    }
};

class CAMERA_kinect_openni : public CAMERA{
public:
    typedef std::shared_ptr<CAMERA_kinect_openni> Ptr;
    typedef const std::shared_ptr<CAMERA_kinect_openni> ConstPtr;

    CAMERA_kinect_openni(){
    }

    void init() override;
    void camera_topics_start_publishing_cb(const sensor_msgs::Image::ConstPtr& depth_msgs){
        _global_parameters.set_camera_topics_status(true);
    }
};

class CAMERA_kinect_v2 : public CAMERA{
public:
    typedef std::shared_ptr<CAMERA_kinect_v2> Ptr;
    typedef const std::shared_ptr<CAMERA_kinect_v2> ConstPtr;

    CAMERA_kinect_v2(){
    }

    void init() override;
    void camera_topics_start_publishing_cb(const sensor_msgs::Image::ConstPtr& depth_msgs){
        _global_parameters.set_camera_topics_status(true);
    }
};


class ROBOT{
public:
    typedef std::shared_ptr<ROBOT> Ptr;
    typedef const std::shared_ptr<ROBOT> ConstPtr;

    ROBOT(){}

    virtual void init() = 0;

protected:
    ros::NodeHandle _nh_robot;
    std::unique_ptr<ros::AsyncSpinner> _robot_spinner;
    std::shared_ptr<ros::Subscriber> _eef_state_sub;
};

class BAXTER : public ROBOT{
public:
    typedef std::shared_ptr<BAXTER> Ptr;
    typedef const std::shared_ptr<BAXTER> ConstPtr;

    BAXTER(){}

    void init() override;

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

class CRUSTCRAWLER : public ROBOT {
public:
    typedef std::shared_ptr<CRUSTCRAWLER> Ptr;
    typedef const std::shared_ptr<CRUSTCRAWLER> ConstPtr;

    CRUSTCRAWLER(){}
    void init() override;

    //call back that register baxter left end effector pose and rearrange the orientation in RPY
    void eef_Callback(const crustcrawler_core_msgs::EndpointState::ConstPtr& eef_feedback){
        calibration_helpers_methods::locate_eef_pose(eef_feedback->pose, _global_parameters);
        //ROS_WARN_STREAM("locating eef stuff gave for position: " << _global_parameters.get_robot_eef_position()
          //              << "\n and for orientation: " << _global_parameters.get_robot_eef_rpy_orientation());
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

    void show_image();
    void acquire_points();
    void acquire_optitrack_points();
    bool solve_for_transformation_matrix();
    void tf_base_conversion(Eigen::Vector3d& converted_point,
                            Eigen::Vector3d& to_be_converted_point,
                            const std::string& child, const std::string& parent);


    Calibration_Data_config& get_global_parameters(){
        return _global_parameters;
    }
private:
    ros::NodeHandle _nh_clibrator;
    std::unique_ptr<ros::AsyncSpinner> _calibrator_spinner;
    ROBOT::Ptr _robot;
    CAMERA::Ptr _camera;
};

}

#endif //_BAXTER_MOVER_HPP
