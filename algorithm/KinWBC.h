#pragma once

#include <Eigen/Dense>
#include "useful_math.h"
#include <utility>
#include <vector>
#include <string>
#include <iostream>
#include "data_type.h"

#include "robot_wrapper.h"

struct Task {

    std::string taskName;

    int ee_index; // end-effector index to access Jacobian, position, velocity from robotwrapper
    
    VectorXd dxDes, ddxDes; //task space desired velocity and acceleration 
    VectorXd delta_q, dq, ddq; //joint space velocity and acceleration 
    MatrixXd J, dJ, Jpre; // Jacobian, Jacobian derivative,...
    MatrixXd N; // Null space projection matrix of J
    MatrixXd kp, kd; // pd gain in task space
    Eigen::DiagonalMatrix<double, -1> W; //weighted matrix for pseudo inverse
    VectorXd errX, derrX; // task space error and derivative

    Task(std::string name, int eeId) {taskName = name; ee_index = eeId;}; // constructor
};

class KinWBC {
public:
    // Constructor
    KinWBC ();
    std::vector<Task> kin_task_stand;
    // std::vector<Task> kin_task_walk;
    Eigen::VectorXd out_delta_q, out_dq, out_ddq;

    void printTaskInfo();
    void computeWBC_IK (const RobotWrapper& robot_wrapper);
};