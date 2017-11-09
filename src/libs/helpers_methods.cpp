#include "helpers_methods.hpp"

void calibration_helpers_methods::locate_eef_pose(geometry_msgs::Pose eef_feedback, Data_config &parameters){
    Eigen::VectorXd end_effector_pose(6);
    geometry_msgs::Pose eef_pose_quat = eef_feedback;
    tf::Quaternion eef_rpy_orientation;

    tf::quaternionMsgToTF(eef_pose_quat.orientation, eef_rpy_orientation);

    double roll, yaw, pitch;
    tf::Matrix3x3 m(eef_rpy_orientation);
    m.getRPY(roll, pitch, yaw);
    Eigen::Vector3d eef_current_position;
    Eigen::Vector3d eef_current_orientation;
    eef_current_position << eef_pose_quat.position.x,
            eef_pose_quat.position.y,
            eef_pose_quat.position.z;

    eef_current_orientation <<    roll,
            pitch,
            yaw;
    end_effector_pose << eef_pose_quat.position.x,
            eef_pose_quat.position.y,
            eef_pose_quat.position.z,
            roll,
            pitch,
            yaw;
    parameters.set_robot_eef_position(eef_current_position);
    parameters.set_robot_eef_rpy_orientation(eef_current_orientation);
    parameters.set_robot_eef_pose(eef_pose_quat);
    parameters.set_robot_eef_rpy_pose(end_effector_pose);
    //ROS_WARN_STREAM("locating eef stuff gave for position: " << eef_current_position << "\n and for orientation: " << eef_current_orientation);
}

void calibration_helpers_methods::locate_optitrack_marker_position(const geometry_msgs::PoseStamped::ConstPtr& optitrack_feedback,
                                                                   Data_config &parameters){
    parameters.set_optitrack_marker_msg(optitrack_feedback);
}

//get largest difference between elements of two vectors
double calibration_helpers_methods::largest_difference(std::vector<double> &first, std::vector<double> &second){
        Eigen::VectorXd difference(first.size());
        double my_max = 0;
        for(size_t j = 0; j < first.size(); ++j)
            difference(j) = fabs(first[j] - second[j]);
        for(size_t j = 0; j < first.size(); ++j){
                if(difference(j) > my_max)
                    my_max = difference(j);
            }
        return my_max;
    }
