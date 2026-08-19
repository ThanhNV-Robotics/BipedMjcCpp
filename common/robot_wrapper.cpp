#include "robot_wrapper.h"
#include "data_type.h"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/fwd.hpp>

//  Openning note: 
//  For Pinocchio: The base translation part is expressed in the parent frame (here the world coordinate system)
//  while its velocity is expressed in the body coordinate system.
//  https://github.com/stack-of-tasks/pinocchio/issues/1137
//  q = [global_base_position, global_base_quaternion, joint_positions]
//  v = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]
//  as base linear 

namespace pin = pinocchio;

namespace {
// Loads the URDF into a pinocchio::Model, adding a floating joint for the
// biped's free-flyer base. Used directly in the member-initializer list, so
// pin_model_ can be built before the const model_* attributes are derived
// from it.
pin::Model buildPinocchioModel(const std::string& urdf_path)
{
    pin::Model model;
    pin::JointModelFreeFlyer root_joint; 
    try
    {
        pin::urdf::buildModel(urdf_path, root_joint, model); // add a floating base joint here
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to load URDF: " << urdf_path << std::endl;
        std::cerr << "Pinocchio error: " << e.what() << std::endl;

        throw;  // Re-throw the exception
    }
    return model;
}

int countActuatedJoints(const pin::Model& model)
{
    int na = 0;
    for (pin::JointIndex j_index = 0; j_index < model.njoints; ++j_index)
    {
        const auto& jtype = model.joints[j_index].shortname();

        // skip the floating joint/base
        if (jtype == "JointModelFreeFlyer") continue;
        //skip the universe joint, which pinocchio automatically adds
        if (model.names[j_index] == "universe") continue;

        ++na;
    }
    return na;
}
} 

// Constructor
RobotWrapper::RobotWrapper(const std::string& urdf_path)
{
    //*********************************************** */
    // model info
    pin_model_ = buildPinocchioModel(urdf_path); // use this function for floating base
    pin_data_ = pin::Data(pin_model_);
    model_nq_ = pin_model_.nq;
    model_nv_ = pin_model_.nv;
    model_njoint_ = pin_model_.njoints;
    model_na_ = countActuatedJoints(pin_model_);

    pinocchio::urdf::buildModel(urdf_path, model_fixedbase_);
    data_fixedbase_ = pin::Data(model_fixedbase_);

    //*********************************************** */
    // accessible terms
    // configuration space
    q   = pin::neutral(pin_model_); // size nq, valid neutral config (identity quaternion)
    dq  = VectorXd::Zero(model_nv_);
    ddq = VectorXd::Zero(model_nv_);

    J_Rfeet_W = Jacobian6::Zero(6, model_nv_);
    J_Lfeet_W = Jacobian6::Zero(6, model_nv_);
    J_base_W  = Jacobian6::Zero(6, model_nv_);
    Jcom_W    = Jacobian3::Zero(3, model_nv_);

    pos_R_feet_W = Vector3d::Zero();
    pos_L_feet_W = Vector3d::Zero();
    pos_base_W   = Vector3d::Zero();
    pos_R_feet_B = Vector3d::Zero();
    pos_L_feet_B = Vector3d::Zero();
    pos_base_B   = Vector3d::Zero();

    rot_R_feet_W = Matrix3d::Identity();
    rot_L_feet_W = Matrix3d::Identity();
    rot_R_feet_B = Matrix3d::Identity();
    rot_L_feet_B = Matrix3d::Identity();

    vel_R_feet_B = Vector3d::Zero();
    vel_L_feet_B = Vector3d::Zero();

    // assign left and right leg joint index
    for (pin::JointIndex j_index = 0; j_index < this->model_njoint_; ++j_index)
    {
        const std::string& joint_name = this->pin_model_.names[j_index];

        if (joint_name.find("left") != std::string::npos)
        {
            left_leg_joint_ids_.push_back(j_index);
        }

        if (joint_name.find("right") != std::string::npos)
        {
            right_leg_joint_ids_.push_back(j_index);
        }
    }

    // assign joint limit
    this->min_joint_pos_ = pin_model_.lowerPositionLimit.tail(model_na_);
    this->max_joint_pos_ = pin_model_.upperPositionLimit.tail(model_na_);
    this->joint_vel_limit_ = pin_model_.velocityLimit.tail(model_na_);
    this->joint_torque_limit_ = pin_model_.effortLimit.tail(model_na_);
}

void RobotWrapper::updateFixedBaseState(RobotConfiguration rb_cf, RobotSpatialVelocity rb_v)
{
    return;
}

void RobotWrapper::updateRobotState (RobotConfiguration q, RobotSpatialVelocity dq)
{
    /**
    * @brief update internal configuration vector q and velocity dq
    *  q = [base_position_in_world, base_quaternion_in_world, joint_position], dim = 3 + 4 +12 = nq
    *  dq = [base_linear_vel_in_World, base_angular_vel_in_world, joint vel], dim nv
    */

    // update position
    this->q.segment<3>(0) = q.pos_b_W; // base pos
    this->q.segment<4>(3) = q.quat_b_W.coeffs(); // base quaternion
    this->q.segment(7, this->model_na_) = q.qj; // joint position

    // for base linear and angular velocity, we have to transform to local base frame
    // because in pinocchio v = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]
    
    // update base velocity
    this->dq.segment<3>(0) = q.quat_b_W.inverse() * dq.vb_W; // base linear velocity, converted to local base frame
    this->dq.segment<3>(3) = q.quat_b_W.inverse() * dq.wb_W; // base angular velocity, convert to local frame also

    this->dq.segment(6, this->model_na_) = dq.dq_j; // joint velocity
}

void RobotWrapper::computeDyn()
{
    /**
    * @brief Computes all the kinematics and dynamics terms
    *  call updateRobotState first to update q and dq
    */
    
    // compute M
    pin::crba(pin_model_, pin_data_, q);
    // Pinocchio only gives half of the M, needs to restore it here
    pin_data_.M.triangularView<Eigen::Lower>() = pin_data_.M.transpose().triangularView<Eigen::Lower>();
    dyn_M = pin_data_.M;

    // cal Minv
    pinocchio::computeMinverse(pin_model_, pin_data_, q);
    pin_data_.Minv.triangularView<Eigen::Lower>() = pin_data_.Minv.transpose().triangularView<Eigen::Lower>();
    dyn_M_inv = pin_data_.Minv;

    // cal C
    pinocchio::computeCoriolisMatrix(pin_model_, pin_data_, q, dq);
    dyn_C = pin_data_.C;

    // cal G
    pinocchio::computeGeneralizedGravity(pin_model_, pin_data_, q);
    dyn_G = pin_data_.g;

    // cal Ag, Centroidal Momentum Matrix. First three rows: linear, other three rows: angular
    pinocchio::dccrba(pin_model_, pin_data_, q, dq);
    pinocchio::computeCentroidalMomentum(pin_model_, pin_data_, q, dq);
    dyn_Ag = pin_data_.Ag;
    dyn_dAg = pin_data_.dAg;

    // cal nonlinear item
    dyn_Non = dyn_C * dq + dyn_G;

    // cal I
    pinocchio::ccrba(pin_model_, pin_data_, q, dq);
    // inertia = pin_data_.Ig.inertia().matrix();

    // cal CoM
    CoM_pos = pin_data_.com[0];
}

void RobotWrapper::computeJacobiansandPosition() // Compute J_lf(q)
{
    /**
    * @brief compute all the Jacobians and positions
    */
    pin::forwardKinematics(pin_model_, pin_data_, q);
    pin::computeJointJacobians(pin_model_, pin_data_, q);

    pin::getJointJacobian(pin_model_,pin_data_ , left_leg_joint_ids_.back() , pinocchio::LOCAL_WORLD_ALIGNED , J_Lfeet_W);
    pin::getJointJacobian(pin_model_,pin_data_ , right_leg_joint_ids_.back() , pinocchio::LOCAL_WORLD_ALIGNED , J_Rfeet_W);
    pin::getJointJacobian(pin_model_, pin_data_, 1, pinocchio::LOCAL_WORLD_ALIGNED, J_base_W);
    Jcom_W = pin_data_.Jcom;

    // Frame position in World Frame
    pos_R_feet_W = pin_data_.oMi[right_leg_joint_ids_.back()].translation(); // right feet position
    pos_L_feet_W = pin_data_.oMi[left_leg_joint_ids_.back()].translation();
    pos_base_W = pin_data_.oMi[0].translation();

    // Frame position in LOCAL BASE frame
    

}

void RobotWrapper::printModelInfo()
{
    // print some basic model info
    std::cout << "\n========== Pinocchio Model Info ==========\n";

    // Basic model dimensions
    std::cout << "Number of joints (njoints): " << this->model_njoint_ << '\n';
    std::cout << "Number of positions (nq):   " << this->model_nq_ << '\n';
    std::cout << "Number of velocities (nv):  " << this->model_nv_ << '\n';
    std::cout << "Number of actuated joint (na):  " << this->model_na_ << '\n';

    std::cout << "\n------------------------------------------\n";
    std::cout << "Joints:\n";

    for (pin::JointIndex joint_id = 0; joint_id < pin_model_.njoints; ++joint_id)
    {
        const auto joint = pin_model_.joints[joint_id];
        std::cout << "ID: " << joint_id
                  << " | Name: " << pin_model_.names[joint_id]
                  << " | Type: " << joint.shortname()
                  << " | nq: " << joint.nq()
                  << " | nv: " << joint.nv()
                  << " | Parent: " << pin_model_.parents[joint_id]
                  << '\n';
    }

    std::cout << "\n------------------------------------------\n";
    std::cout << "Left Leg Joints:\n";
    for (const auto& joint_id : left_leg_joint_ids_)
    {
        const auto joint = pin_model_.joints[joint_id];
        std::cout << "ID: " << joint_id
                  << " | Name: " << pin_model_.names[joint_id]
                  << " | Type: " << joint.shortname()
                  << " | Parent: " << pin_model_.parents[joint_id]
                  << '\n';
    }

    std::cout << "\n------------------------------------------\n";
    std::cout << "Right Leg Joints:\n";
    for (const auto& joint_id : right_leg_joint_ids_)
    {
        const auto joint = pin_model_.joints[joint_id];
        std::cout << "ID: " << joint_id
                  << " | Name: " << pin_model_.names[joint_id]
                  << " | Type: " << joint.shortname()
                  << " | Parent: " << pin_model_.parents[joint_id]
                  << '\n';
    }
}

void RobotWrapper::printFixedBaseModelInfo ()
{
    // print some basic model info
    std::cout << "\n========== Pinocchio Fixed-Base Model Info ==========\n";

    // Basic model dimensions
    std::cout << "Number of joints (njoints): " << model_fixedbase_.njoints << '\n';
    std::cout << "Number of positions (nq):   " << model_fixedbase_.nq << '\n';
    std::cout << "Number of velocities (nv):  " << model_fixedbase_.nv << '\n';
    std::cout << "Number of actuated joint (na):  " << countActuatedJoints(model_fixedbase_) << '\n';

    std::cout << "\n------------------------------------------\n";
    std::cout << "Joints:\n";

    for (pin::JointIndex joint_id = 0; joint_id < model_fixedbase_.njoints; ++joint_id)
    {
        const auto joint = model_fixedbase_.joints[joint_id];
        std::cout << "ID: " << joint_id
                  << " | Name: " << model_fixedbase_.names[joint_id]
                  << " | Type: " << joint.shortname()
                  << " | nq: " << joint.nq()
                  << " | nv: " << joint.nv()
                  << " | Parent: " << model_fixedbase_.parents[joint_id]
                  << '\n';
    }
}