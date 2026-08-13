#include "robot_wrapper.h"
#include "data_type.h"
#include "iostream"
#include <string>
#include <random>

#include <pinocchio/parsers/urdf.hpp>

namespace pin = pinocchio;
using namespace std;

const string URDF_PATH = "models/urdf/biped_robot_12dof.urdf";

int main ()
{
    cout<<"Test robot wrapper"<<endl;

    RobotWrapper robot_wrapper(URDF_PATH);
    robot_wrapper.printModelInfo();

    // Test Jacobian given a random joint configuration
    const std::vector<double> base_pos = {0.0, 0.0, 0.8}; //assume base position

    // RobotWrapper doesn't expose its pinocchio::Model, so load one here just
    // to read the leg joints' position/velocity limits from the URDF.
    pin::Model pin_model;
    pin::urdf::buildModel(URDF_PATH, pin::JointModelFreeFlyer(), pin_model);
    const int na = pin_model.nv - 6; // number of actuated leg joints

    // generate a random legs' joint position and velocity within the limit
    std::random_device rd;
    std::mt19937 gen(rd());

    JointState joint_state(na);
    for (int i = 0; i < na; ++i)
    {
        std::uniform_real_distribution<double> pos_dist(
            pin_model.lowerPositionLimit[7 + i], pin_model.upperPositionLimit[7 + i]);
        joint_state.qj[i] = pos_dist(gen);

        std::uniform_real_distribution<double> vel_dist(
            -pin_model.velocityLimit[6 + i], pin_model.velocityLimit[6 + i]);
        joint_state.dqj[i] = vel_dist(gen);
    }

    IMUSensor imu_sensor; // identity base orientation

    // compute and print out a foot Jacobian
    Jacobian6 J = robot_wrapper.computeLeftFeetJointJacobianGlobal(joint_state, imu_sensor);
    cout << "\nLeft foot Jacobian (6 x " << J.cols() << "):\n" << J << endl;

    return 0;
}