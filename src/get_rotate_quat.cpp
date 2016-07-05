//subscribe to rgb input and look in the incoming stream for a a certain object (baxter gripper)
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

double qx, qy, qz, qw;
Eigen::MatrixXd quaternoin_rot(3,3);


int main(int argc, char** argv)
{
    qx = atof(argv[1]); qy = atof(argv[2]); qz = atof(argv[3]); qw = atof(argv[4]);

    std::cout << "the element are: " << qx << " " << qy << " " << qz << " " << qw << " " << std::endl;
    std::cout << "the sum is: " << pow(qx,2) + pow(qy,2) + pow(qz,2) + pow(qw,2) << std::endl;
    quaternoin_rot << -2*(pow(qx,2) + pow(qy,2)) + 1,             2*(qy*qx - qz*qw),             2*(qy*qw + qx*qz),
                                  2*(qy*qx + qz*qw), -2*(pow(qx,2) + pow(qz,2)) + 1,             2*(qz*qy - qx*qw),
                                  2*(-qy*qw + qx*qz),             2*(qz*qy + qx*qw), -2*(pow(qx,2) + pow(qy,2)) + 1;
    std::cout << "here is the quaternion rotation matrix: " << std::endl << quaternoin_rot << std::endl;
    return 0;
}
