#ifndef __PARAMETERS_HPP__
#define __PARAMETERS_HPP__
#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
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
#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <baxter_core_msgs/EndpointState.h>
#include <crustcrawler_core_msgs/EndpointState.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <Eigen/Core>
#include <moveit/move_group/move_group_context.h>
#include <moveit/planning_pipeline/planning_pipeline.h>

#include <yaml-cpp/yaml.h>

struct Calibration_Parameters {
    XmlRpc::XmlRpcValue parameters;
    geometry_msgs::Pose robot_eef_pose;
    Eigen::VectorXd robot_eef_rpy_pose;
    Eigen::Vector3d robot_eef_position;
    Eigen::Vector3d robot_eef_rpy_orientation;
    Eigen::Vector4d quaternion_vector;
    int number_of_points = 10, number_of_valid_point = 0;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat raw_input_picture;
    sensor_msgs::ImageConstPtr rgb_msg, depth_msg;
    sensor_msgs::CameraInfoConstPtr camera_info_msg;
    geometry_msgs::PoseStamped::ConstPtr optitrack_marker_msg;
    std::vector<Eigen::Vector2i> marker_center;
    std::vector<double> last_eef_position;
    double x_coord = 0.0, y_coord = 0.0, z_coord = 0.0, epsilon = 100;
    Eigen::Matrix4d point_in_robot_frame, point_in_camera_frame;
    aruco::MarkerDetector marker_detector;
    aruco::CameraParameters camera_character;
    std::vector<aruco::Marker> markers;
    float marker_size = 0.04;

    Eigen::VectorXd baxter_left_arm_joints_positions, baxter_right_arm_joints_positions;

    std::shared_ptr<rgbd_utils::RGBD_to_Pointcloud> converter;
    sensor_msgs::PointCloud2 ptcl_msg;
    Eigen::MatrixXd markers_positions_camera_frame, markers_positions_robot_frame;
    Eigen::Matrix4d transformation_matrix;
    Eigen::Vector3d transformation_RPY, transformation_position;
    tf::Quaternion transformation_quaternion;

    bool camera_topics_good = false, first_iteration = true;

};
class Calibration_Data_config{
public:
    Calibration_Parameters params;
    Calibration_Data_config(){
        //params.point_in_camera_frame.resize(params.number_of_points, 4);
        //params.point_in_robot_frame.resize(params.number_of_points, 4);
    }
    ///getters
    XmlRpc::XmlRpcValue& get_parameters(){
        return params.parameters;
    }

    int get_number_of_points(){
        return params.number_of_points;
    }

    geometry_msgs::Pose& get_robot_eef_pose(){
        return params.robot_eef_pose;
    }

    Eigen::VectorXd& get_robot_rpy_pose(){
        return params.robot_eef_rpy_pose;
    }

    Eigen::Vector3d get_robot_eef_position(){
        return params.robot_eef_position;
    }

    Eigen::Vector3d get_robot_eef_rpy_orientation(){
        return params.robot_eef_rpy_orientation;
    }

    cv_bridge::CvImagePtr& get_cvt(){
        return params.cv_ptr;
    }

    cv::Mat& get_raw_original_picture(){
        return params.raw_input_picture;
    }

    geometry_msgs::PoseStamped::ConstPtr& get_optitrack_marker_msg(){
        return params.optitrack_marker_msg;
    }

    sensor_msgs::ImageConstPtr& get_rgb_msg(){
        return params.rgb_msg;
    }

    sensor_msgs::ImageConstPtr& get_depth_msg(){
        return params.depth_msg;
    }

    sensor_msgs::CameraInfoConstPtr& get_camera_info_msg(){
        return params.camera_info_msg;
    }

    std::vector<Eigen::Vector2i>& get_marker_centers(){
        return params.marker_center;
    }

    std::vector<double>& get_last_eef_position(){
        return params.last_eef_position;
    }

    double get_x_coordinate(){
        return params.x_coord;
    }

    double get_y_coordinate(){
        return params.y_coord;
    }

    double get_z_coordinate(){
        return params.z_coord;
    }

    double get_epsilon(){
        return params.epsilon;
    }

    Eigen::Vector3d get_marker_position(){
        Eigen::Vector3d marker_position;
        marker_position << params.x_coord, params.y_coord, params.z_coord;
        return marker_position;
    }

    Eigen::Matrix4d& get_point_in_robot_frame(){
        return params.point_in_robot_frame;
    }

    Eigen::Matrix4d& get_point_in_camera_frame(){
        return params.point_in_camera_frame;
    }

    aruco::CameraParameters& get_camera_character(){
        return params.camera_character;
    }

    aruco::MarkerDetector& get_aruco_marker_detector(){
        return params.marker_detector;
    }

    std::vector<aruco::Marker>& get_markers(){
        return params.markers;
    }

    float& get_marker_size(){
        return params.marker_size;
    }

    rgbd_utils::RGBD_to_Pointcloud& get_rgb_cloud_converter(){
        return *params.converter;
    }

    sensor_msgs::PointCloud2& get_pointcloud_msg(){
        return params.ptcl_msg;
    }

    Eigen::MatrixXd& get_markers_positions_camera_frame(){
        return params.markers_positions_camera_frame;
    }

    Eigen::MatrixXd& get_markers_positions_robot_frame(){
        return params.markers_positions_robot_frame;
    }

    Eigen::Matrix4d& get_transformation_matrix(){
        return params.transformation_matrix;
    }

    Eigen::Vector3d& get_transformation_RPY(){
        return params.transformation_RPY;
    }

    Eigen::Vector3d& get_transformation_position(){
        return params.transformation_position;
    }

    tf::Quaternion& get_transformation_quaternion(){
        return params.transformation_quaternion;
    }

    Eigen::Vector4d& get_transformation_quaternion_vector(){
        params.quaternion_vector << params.transformation_quaternion.getX(), params.transformation_quaternion.getY(),
                params.transformation_quaternion.getZ(), params.transformation_quaternion.getW();
        return params.quaternion_vector;
    }

    bool get_camera_topics_status(){
        return params.camera_topics_good;
    }

    bool get_first_iteration(){
        return params.first_iteration;
    }

    int& get_number_of_validated_points(){
        return params.number_of_valid_point;
    }

    ///setters
    ///
    void set_parameters(XmlRpc::XmlRpcValue parameters){
        params.parameters = parameters;
    }

    void set_number_of_points(int number_points){
        params.number_of_points = number_points;
        params.markers_positions_camera_frame.resize(params.number_of_points, 4);
        params.markers_positions_robot_frame.resize(params.number_of_points, 4);
    }

    void set_robot_eef_pose(geometry_msgs::Pose& eef_pose){
            params.robot_eef_pose = eef_pose;
    }

    void set_robot_eef_rpy_pose(Eigen::VectorXd& eef_rpy_pose){
            params.robot_eef_rpy_pose = eef_rpy_pose;
    }

    void set_robot_eef_position(Eigen::Vector3d& eef_position){
            params.robot_eef_position = eef_position;
    }

    void set_robot_eef_rpy_orientation(Eigen::Vector3d& eef_rpy_orientation){
            params.robot_eef_rpy_orientation = eef_rpy_orientation;
    }

    void set_raw_original_picture(cv::Mat& picture){
        params.raw_input_picture = picture;
    }

    void set_optitrack_marker_msg(const geometry_msgs::PoseStamped::ConstPtr& optitrack_marker_msg){
        params.optitrack_marker_msg = optitrack_marker_msg;
    }

    void set_rgb_msg(sensor_msgs::ImageConstPtr& input_rgb_msg){
        params.rgb_msg = input_rgb_msg;
    }

    void set_depth_msg(sensor_msgs::ImageConstPtr& input_depth_msg){
        params.depth_msg = input_depth_msg;
    }

    void set_camera_info_msg(aruco::CameraParameters& camera_character){
        params.camera_character = camera_character;
    }

    void set_marker_center(std::vector<Eigen::Vector2i>& vector_marker_center){
        params.marker_center = vector_marker_center;
    }

    void set_last_eef_position(std::vector<double>& eef_position){
        params.last_eef_position = eef_position;
    }

    void set_x_coordinate(double x){
        params.x_coord = x;
    }

    void set_y_coordinate(double y){
        params.y_coord = y;
    }

    void set_z_coordinate(double z){
        params.z_coord = z;
    }

    void set_epsilon(double epsilon){
        params.epsilon = epsilon;
    }

    void set_point_in_robot_frame(Eigen::Matrix4d& point){
        params.point_in_robot_frame = point;
    }

    void set_point_in_camera_frame(Eigen::Matrix4d& point){
        params.point_in_camera_frame = point;
    }

    void set_camera_character(aruco::CameraParameters& camera_character){
        params.camera_character = camera_character;
    }

    void set_markers(std::vector<aruco::Marker>& markers){
        params.markers = markers;
    }

    void set_marker_size(float marker_size){
        params.marker_size = marker_size;
    }

    void set_rgb_cloud_converter(sensor_msgs::ImageConstPtr depth_msg,
                                 sensor_msgs::ImageConstPtr rgb_msg,
                                 sensor_msgs::CameraInfoConstPtr camera_info){
        params.converter.reset(new rgbd_utils::RGBD_to_Pointcloud(depth_msg, rgb_msg, camera_info));
    }

    void set_pointcloud_msg(sensor_msgs::PointCloud2 ptcl_msg){
        params.ptcl_msg = ptcl_msg;
    }

    void set_marker_position_camera_frame(Eigen::MatrixX4d markers_positions){
        params.markers_positions_camera_frame = markers_positions;
    }

    void set_marker_position_robot_frame(Eigen::MatrixX4d markers_positions){
        params.markers_positions_robot_frame = markers_positions;
    }

    void set_transformation_matrix(Eigen::Matrix4d trans_matrix){
        params.transformation_matrix = trans_matrix;
    }

    void set_transformation_RPY(Eigen::Vector3d RPY){
        params.transformation_RPY = RPY;
    }

    void set_transformation_position(Eigen::Vector3d position){
        params.transformation_position = position;
    }

    void set_transformation_quaternion(tf::Quaternion quat){
        params.transformation_quaternion = quat;
    }

    void set_camera_topics_status(bool status){
        params.camera_topics_good = status;
    }

    void set_first_iteration(bool first){
        params.first_iteration = first;
    }

    void set_number_of_validated_point(int i){
        params.number_of_valid_point = i;
    }
};

#endif
