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

} // namespace

RobotWrapper::RobotWrapper(const std::string& urdf_path)
    : pin_model_(buildPinocchioModel(urdf_path))
    , pin_data_(pin_model_)
    , model_nq_(pin_model_.nq)
    , model_nv_(pin_model_.nv)
    , model_njoint_(pin_model_.njoints)
    , model_na_(countActuatedJoints(pin_model_))
    , joint_state_(model_na_)
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
}

//  Impportant note: 
//  For Pinocchio: The base translation part is expressed in the parent frame (here the world coordinate system)
//  while its velocity is expressed in the body coordinate system.
//  https://github.com/stack-of-tasks/pinocchio/issues/1137
//  q = [global_base_position, global_base_quaternion, joint_positions]
//  v = [local_base_velocity_linear, local_base_velocity_angular, joint_velocities]

Jacobian6 RobotWrapper::computeLeftFeetJointJacobianGlobal(JointState& joint_state, IMUSensor& imu_sensor)
{
    /**
    * @brief Computes the 6D Jacobian of the left foot in global (LOCAL_WORLD_ALIGNED) frame.
    * 
    * @param joint_state Joint position, velocity, and torque data (uses qj for actuated joint positions).
    * @param imu_sensor IMU sensor measurements (uses imu_quat_ for base orientation).
    * @return Jacobian6 The 6xnv foot Jacobian matrix.
    */

    // all joint position, include the base position
    VectorXd q = VectorXd::Zero(pin_model_.nq); // dim nq

    // 1. Fill base orientation from IMUSensor
    Quat base_imu_quat = imu_sensor.imu_quat_;
    q.segment<4>(3) = base_imu_quat.coeffs();

    // 2. Fill joint (actuated) position from JointState
    q.segment(7, model_na_) = joint_state.qj;

    // 3. Compute Jacobian
    pin::computeJointJacobians(pin_model_, pin_data_, q);

    pin::JointIndex joint_id = this->left_leg_joint_ids_.back(); // get last joint id -> ankle joint
    Jacobian6 J = Jacobian6::Zero(6, pin_model_.nv);
    pin::getJointJacobian(pin_model_, pin_data_, joint_id, pin::LOCAL_WORLD_ALIGNED, J);

    return J;
}

void RobotWrapper::updateJointState(JointState& jointStateIn)
{
    this->joint_state_ = jointStateIn;
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