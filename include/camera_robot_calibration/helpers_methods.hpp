#include "parameters.hpp"

namespace calibration_helpers_methods{
/*get crustcrawler eef pose with orientation expressed as RPY
 * input: pose as geometry msgs of the crustcrawler eef, and the name of the gripper
 * return: nothing but set the corresponding variable in the Data_config class
 * */
void locate_eef_pose(geometry_msgs::Pose eef_feedback, Data_config& parameters);

}
