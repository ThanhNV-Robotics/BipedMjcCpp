
#include "data_type.h"
#include "iostream"
#include "robot_wrapper.h"
#include "test_mj_pin_dyn.h"
#include "matplotlibcpp.h"

namespace pin = pinocchio;
using namespace std;

namespace plt = matplotlibcpp;

const string URDF_PATH = "models/urdf/biped_robot_12dof.urdf";
const string XML_PATH = "models/mjcf/biped_robot_floatingbase_12dof.xml";
const string LEFT_FOOT_BODY_NAME = "left_ankle_pitch_link";
const string FREE_JOINT_NAME = "floating_base_joint";
const double PI = 3.14;

int main ()
{
    cout<<"Test robot wrapper"<<endl;
    TestDyn tester = TestDyn(URDF_PATH, XML_PATH);    
    bool test_result = false;
    test_result = tester.testPinMjcJacobians("left_ankle_pitch_link");
    test_result = tester.testPinMjcJacobians("right_ankle_pitch_link");
    
    test_result = tester.testPinMjcPosition("left_ankle_pitch_link");
    test_result = tester.testPinMjcPosition("right_ankle_pitch_link");

    test_result = tester.testPinMjcOrientation("right_ankle_pitch_link");


    RobotWrapper robot_wrapper = RobotWrapper(URDF_PATH);
    RobotConfiguration q = tester.generateRandomConfiguration(robot_wrapper);
    // Generate a random robot spatial velocity    
    RobotSpatialVelocity v = tester.generateRandomRobotSpatialVelocity(robot_wrapper);

    robot_wrapper.updateRobotState(q, v);
    robot_wrapper.computeKin();

    MatrixXd J_base_W;
    J_base_W = robot_wrapper.J_base_W;
    cout<<"Base Jacobian Matrix:"<<endl;
    cout<<J_base_W<<endl;

}