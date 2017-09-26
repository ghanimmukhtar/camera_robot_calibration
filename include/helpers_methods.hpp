#include "parameters.hpp"

namespace calibration_helpers_methods{
/*Take the end effector topic msgs and save it accordingly in the global param variable
 * input: pose as geometry msgs of the arm eef, and the name of the gripper
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_eef_pose(geometry_msgs::Pose eef_feedback, Data_config& parameters);

/*Take the optitrack marker position msgs and save it accordingly in the global param variable
 * input: pose as geometry msgs of the marke pose, and the name of the gripper
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_optitrack_marker_position(const geometry_msgs::PoseStamped::ConstPtr& optitrack_feedback, Data_config& parameters);

}
