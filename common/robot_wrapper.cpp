#include "robot_wrapper.h"
#include "data_type.h"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/multibody/fwd.hpp>

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
        pin::urdf::buildModel(urdf_path, root_joint, model);
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
    : pin_model_(buildPinocchioModel(urdf_path))
    , pin_data_(pin_model_)
    , model_nq_(pin_model_.nq)
    , model_nv_(pin_model_.nv)
    , model_njoint_(pin_model_.njoints)
    , model_na_(countActuatedJoints(pin_model_))
    , actuated_joint_state_(model_na_)
{
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
    this->min_joint_pos_ = pin_model_.lowerPositionLimit;
    this->max_joint_pos_ = pin_model_.upperPositionLimit;
    this->joint_vel_limit_ = pin_model_.velocityLimit;
    this->joint_torque_limit_ = pin_model_.effortLimit;

    // Robot configuration
    this->q_ = RobotConfiguration(model_na_);
    this->qj_ = ActuatedJointState(model_na_);
}

//  Impportant note: 
//  For Pinocchio: The base translation part is expressed in the parent frame (here the world coordinate system)
//  while its velocity is expressed in the body coordinate system.
//  https://github.com/stack-of-tasks/pinocchio/issues/1137
//  q = [global_base_position, global_base_quaternion, joint_positions]
//  v = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]

Jacobian6 RobotWrapper::computeLeftFeetJointJacobianGlobal(RobotConfiguration& q) // Compute J_lf(q)
{
    /**
    * @brief Computes the 6D Jacobian of the left foot in global (LOCAL_WORLD_ALIGNED) frame.
    * 
    * @param q Robot configuration containing base position, orientation, and actuated joint positions.
    * @return Jacobian6 The 6xnv foot Jacobian matrix.
    */

    // Construct full Pinocchio configuration vector (nq = 7 + na)
    VectorXd q_pin = VectorXd::Zero(pin_model_.nq);

    // 1. Base position
    q_pin.segment<3>(0) = q.qb;

    // 2. Base orientation (quaternion [x, y, z, w])
    q_pin.segment<4>(3) = q.qb_quat.coeffs();

    // 3. Actuated joint positions
    q_pin.segment(7, model_na_) = q.qj;

    // 4. Compute kinematics and Jacobian
    pin::computeJointJacobians(pin_model_, pin_data_, q_pin);

    pin::JointIndex joint_id = this->left_leg_joint_ids_.back(); // get last joint id -> ankle joint
    Jacobian6 J = Jacobian6::Zero(6, pin_model_.nv);
    pin::getJointJacobian(pin_model_, pin_data_, joint_id, pin::LOCAL_WORLD_ALIGNED, J);

    return J;
}

void RobotWrapper::updateJointState(ActuatedJointState& jointStateIn)
{
    this->actuated_joint_state_ = jointStateIn;
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