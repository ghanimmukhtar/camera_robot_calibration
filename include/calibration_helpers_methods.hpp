#include "calibration_parameters.hpp"

namespace calibration_helpers_methods{
/*Take the end effector topic msgs and save it accordingly in the global param variable
 * input: pose as geometry msgs of the arm eef, and the name of the gripper
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_eef_pose(geometry_msgs::Pose eef_feedback, Calibration_Data_config& parameters);

/*Take the optitrack marker position msgs and save it accordingly in the global param variable
 * input: pose as geometry msgs of the marke pose, and the name of the gripper
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_optitrack_marker_position(const geometry_msgs::PoseStamped::ConstPtr& optitrack_feedback, Calibration_Data_config& parameters);

/* Gives the largest difference between two vetctors elements
 * input: the to compare
 * return: a double that represents the biggest difference found between the two vectors elements
 * */
double largest_difference(std::vector<double> &first, std::vector<double> &second);

/*Deduce crustcrawler end effector pose from TF transform and save it accordingly in the global param variable
 * input: the name of the gripper
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void get_crustcrawler_eef_pose_from_tf(Calibration_Data_config& parameters);
}
