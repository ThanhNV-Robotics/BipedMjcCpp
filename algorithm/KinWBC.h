#pragma once

#include <Eigen/Dense>
#include "useful_math.h"
#include <utility>
#include <vector>
#include <string>
#include <iostream>
#include "data_type.h"

#include "robot_wrapper.h"


#include "joystick_interpreter.h" // for base reference motion
#include "foot_placement.h" // for foot reference motion

struct Task {

    std::string taskName;

    VectorXd X_cur, dX_cur; // feedback in task space
    VectorXd deltaX_des, X_des, dX_des, ddX_des; //task space desired velocity and acceleration 
    VectorXd errX, derrX;

    VectorXd delta_q, dq, ddq; //joint space velocity and acceleration 
    MatrixXd J, dJ, Jpre; // Jacobian, Jacobian derivative,...
    MatrixXd N; // Null space projection matrix of J
    MatrixXd kp, kd; // pd gain in task space
    Eigen::DiagonalMatrix<double, -1> W; //weighted matrix for pseudo inverse

    Task(std::string name) {taskName = name;}; // constructor
};

class KinWBC {
public:
    // Constructor
    KinWBC ();
    // member tasks
    Task task_left_contact  = Task("left_contact"); // input 1 to access to left_feet (jacobian, position, vel)
    Task task_right_contact  = Task("right_contact"); // input 2 to access to right_feet (jacobian, position, vel)
    Task task_CoMXY = Task("CoMXY");  // input 3 to access to Jcom_W in robot_wrapper
    Task task_base_height = Task("base_height"); // input 0 to access to base end-effector
    Task task_base_rpy = Task("base_rpy"); // control base orientation

    Task task_static_contact = Task("static_contact"); // specifically for walking
    Task task_swing_leg = Task("swing_leg");

    std::vector<Task*> kin_task_stand; //using pointer to access the member tasks
    std::vector<Task*> kin_task_walk; 
    // std::vector<Task> kin_task_walk;
    Eigen::VectorXd out_delta_q, out_dq, out_ddq;

    const double dt = 0.001; // sampling time

    void printTaskInfo();
    void updateReference(const JoyStickInterpreter& joyStick, FootPlacement& footPlanner); // get referece from task planner
    void updateCurrent (const RobotWrapper& rb_wrapper); // update current task space estimation
    void computeWBC_IK (const JoyStickInterpreter &joyStick, FootPlacement &footPlanner, const RobotWrapper& robot_wrapper);
};