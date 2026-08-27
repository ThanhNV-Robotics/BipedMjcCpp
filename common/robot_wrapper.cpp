#include "robot_wrapper.h"
#include "data_type.h"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
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

    // J_base-W is always trivaly identity, so just set it one time
    J_base_W  = Jacobian6::Zero(6, model_nv_); 
    J_base_W.block<3, 3>(0, 0) = MatrixXd::Identity(3,3);
    J_base_W.block<3, 3>(3, 3) = MatrixXd::Identity(3,3);

    dJ_base_W = Jacobian6::Zero(6, model_nv_); // fixed as J_base_W = const
    dJ_Lfeet_W = Jacobian6::Zero(6, model_nv_); // must be pre-sized before
    dJ_Rfeet_W = Jacobian6::Zero(6, model_nv_); // getJointJacobianTimeVariation() fills them in computeKin()

    Jcom_W    = Jacobian3::Zero(3, model_nv_);



    pos_R_feet_W = Vector3d::Zero();
    pos_L_feet_W = Vector3d::Zero();
    pos_base_W   = Vector3d::Zero();
    pos_R_feet_B = Vector3d::Zero();
    pos_L_feet_B = Vector3d::Zero();
    pos_base_B   = Vector3d::Zero();
    pos_CoM_W    = Vector3d::Zero();
    vel_base_W   = Vector3d::Zero();

    rot_R_feet_W = Matrix3d::Identity();
    rot_L_feet_W = Matrix3d::Identity();
    rot_R_feet_B = Matrix3d::Identity();
    rot_L_feet_B = Matrix3d::Identity();

    vel_R_feet_W = Vector3d::Zero();
    vel_L_feet_W = Vector3d::Zero();
    vel_R_feet_B = Vector3d::Zero();
    vel_L_feet_B = Vector3d::Zero();

    J_array = { &J_base_W, &J_Lfeet_W, &J_Rfeet_W, &Jcom_W }; //index 0,1,2,3    
    dJ_array = { &dJ_base_W, &dJ_Lfeet_W, &dJ_Rfeet_W };
    pos_array = {&pos_base_W, &pos_L_feet_W, &pos_R_feet_W, &pos_CoM_W};
    vel_array = {&vel_base_W, &vel_L_feet_W, &vel_R_feet_W};



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

    // model_fixedbase_.names[0] is "universe" (Pinocchio's implicit root),
    // so the model_na_ actuated joints start at index 1
    for (int pinoIdx = 0; pinoIdx < model_na_; ++pinoIdx)
    {
        jointNames_.push_back(model_fixedbase_.names[pinoIdx + 1]);
    }
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

void RobotWrapper::integrateConfig (const VectorXd &delta_q)
{
    this->q = pin::integrate(pin_model_, this->q, delta_q);
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

    // Transform to world frame to accept global frame in put

    // Transform Jacobians to accept input dq with base velocity in WORLD frame
    Matrix3d base_rot = pin_data_.oMi[1].rotation();
    MatrixXd Mpj, Mpj_inv;
    Mpj = MatrixXd::Identity(model_nv_, model_nv_);
    Mpj_inv = Eigen::MatrixXd::Identity(model_nv_, model_nv_);
    Mpj.block<3, 3>(0, 0) = base_rot.transpose();
    Mpj.block<3, 3>(3, 3) = base_rot.transpose();
    Mpj_inv.block(0, 0, 3, 3) = base_rot;
    Mpj_inv.block(3, 3, 3, 3) = base_rot;

    dyn_M = Mpj_inv * dyn_M * Mpj;
    dyn_M_inv = Mpj_inv * dyn_M_inv * Mpj;
    dyn_C = Mpj_inv * dyn_C * Mpj;
    dyn_G = Mpj_inv * dyn_G;
    dyn_Non = Mpj_inv * dyn_Non;
}

void RobotWrapper::computeKin() // Compute J_lf(q)
{
    /**
    * @brief compute all the Jacobians, Jacobian derivative and positions
    */
    pin::forwardKinematics(pin_model_, pin_data_, q);
    pin::jacobianCenterOfMass(pin_model_, pin_data_, q, true);
    pin::computeJointJacobians(pin_model_, pin_data_, q);
    pinocchio::computeJointJacobiansTimeVariation(pin_model_, pin_data_, q, dq);

    pin::getJointJacobian(pin_model_,pin_data_ , left_leg_joint_ids_.back() , pinocchio::LOCAL_WORLD_ALIGNED , J_Lfeet_W);
    pin::getJointJacobian(pin_model_,pin_data_ , right_leg_joint_ids_.back() , pinocchio::LOCAL_WORLD_ALIGNED , J_Rfeet_W);
    Jcom_W = pin_data_.Jcom;

    // Foot spatial velocity in world-aligned axes. J_*feet_W is still in
    // LOCAL_WORLD_ALIGNED convention here (not yet reparametrized by Mpj below),
    // so it multiplies directly against the true pinocchio velocity dq.
    vel_L_feet_W = (J_Lfeet_W * dq).head<3>();
    vel_R_feet_W = (J_Rfeet_W * dq).head<3>();
    vel_CoM_W = Jcom_W * dq;

    // Jacobian derivative
    pin::getJointJacobianTimeVariation(pin_model_, pin_data_, left_leg_joint_ids_.back(), pin::LOCAL_WORLD_ALIGNED, dJ_Lfeet_W);
    pin::getJointJacobianTimeVariation(pin_model_, pin_data_, right_leg_joint_ids_.back(), pin::LOCAL_WORLD_ALIGNED, dJ_Rfeet_W);


    // Transform Jacobians to accept input dq with base velocity in WORLD frame
    Matrix3d base_rot = pin_data_.oMi[1].rotation();
    MatrixXd Mpj = MatrixXd::Identity(model_nv_, model_nv_);
    Mpj.block<3, 3>(0, 0) = base_rot.transpose();
    Mpj.block<3, 3>(3, 3) = base_rot.transpose();

    J_Lfeet_W = J_Lfeet_W * Mpj;
    J_Rfeet_W = J_Rfeet_W * Mpj;
    Jcom_W    = Jcom_W    * Mpj;
    dJ_Lfeet_W = dJ_Lfeet_W* Mpj;
    dJ_Rfeet_W = dJ_Rfeet_W* Mpj;

    // Frame position in World Frame. oMi[0] is pinocchio's "universe" joint,
    // always fixed at identity -- the floating base is joint index 1
    pos_L_feet_W = pin_data_.oMi[left_leg_joint_ids_.back()].translation(); // right feet position
    pos_R_feet_W = pin_data_.oMi[right_leg_joint_ids_.back()].translation();
    pos_base_W = pin_data_.oMi[1].translation();
    pos_CoM_W = pin_data_.com[0];
    // dq's base-linear block is in the base's LOCAL frame (see the frame
    // note at the top of this file), rotated to world here by base_rot
    vel_base_W = base_rot * dq.segment<3>(0);

    // Orientation in World frame
    rot_L_feet_W = pin_data_.oMi[left_leg_joint_ids_.back()].rotation();
    rot_R_feet_W = pin_data_.oMi[right_leg_joint_ids_.back()].rotation();

    // compute foot position and velocity w.r.t local base frame
    // forward kinematics for fixed base model
    VectorXd qj_fixedbase = this->q.segment(7, this->model_na_); //extract the actuated joint position only
    VectorXd dqj_fixedbase = this->dq.segment(6, this->model_na_); // extract the actuated joint velocity only
    pin::forwardKinematics(model_fixedbase_, data_fixedbase_, qj_fixedbase, dqj_fixedbase);

    pos_L_feet_B = data_fixedbase_.oMi[left_leg_joint_ids_.back() - 1].translation();
    pos_R_feet_B = data_fixedbase_.oMi[right_leg_joint_ids_.back() - 1].translation();
    rot_L_feet_B = data_fixedbase_.oMi[left_leg_joint_ids_.back() - 1].rotation();
    rot_R_feet_B = data_fixedbase_.oMi[right_leg_joint_ids_.back() - 1].rotation();

    // fixed-base model's frame 0 is the base link itself, so LOCAL_WORLD_ALIGNED
    // here already means "aligned with the local base frame"
    vel_L_feet_B = pin::getVelocity(model_fixedbase_, data_fixedbase_,
                                     left_leg_joint_ids_.back() - 1, pin::LOCAL_WORLD_ALIGNED).linear();
    vel_R_feet_B = pin::getVelocity(model_fixedbase_, data_fixedbase_,
                                     right_leg_joint_ids_.back() - 1, pin::LOCAL_WORLD_ALIGNED).linear();
}

Vector12d RobotWrapper::computeFootInBase (ActuatorState actuator_state)
{
    /**
    * @brief forward kinematics on the fixed-base model to get each foot's
    * position AND velocity w.r.t. the base frame, straight from a raw
    * ActuatorState. Output layout: [left_pos(3), right_pos(3), left_vel(3),
    * right_vel(3)].
    * Reuses the shared data_fixedbase_ cache (also written by computeKin()
    * from this->q) -- calling this overwrites data_fixedbase_ with the FK
    * results for actuator_state.qj/dqj instead, and isn't safe to call
    * concurrently with computeKin() from another thread.
    */
    pin::forwardKinematics(model_fixedbase_, data_fixedbase_, actuator_state.qj, actuator_state.dqj);

    Vector3d left_foot_pos_B = data_fixedbase_.oMi[left_leg_joint_ids_.back() - 1].translation();
    Vector3d right_foot_pos_B = data_fixedbase_.oMi[right_leg_joint_ids_.back() - 1].translation();

    // fixed-base model's frame 0 is the base link itself, so LOCAL_WORLD_ALIGNED
    // here already means "aligned with the local base frame" (same as computeKin())
    Vector3d left_foot_vel_B = pin::getVelocity(model_fixedbase_, data_fixedbase_,
                                                 left_leg_joint_ids_.back() - 1, pin::LOCAL_WORLD_ALIGNED).linear();
    Vector3d right_foot_vel_B = pin::getVelocity(model_fixedbase_, data_fixedbase_,
                                                  right_leg_joint_ids_.back() - 1, pin::LOCAL_WORLD_ALIGNED).linear();

    Vector12d feet_pos_vel_B;
    feet_pos_vel_B << left_foot_pos_B, right_foot_pos_B, left_foot_vel_B, right_foot_vel_B;
    return feet_pos_vel_B;
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

RobotWrapper::IkRes RobotWrapper::computeInK_Leg(const Eigen::Matrix3d &Rdes_L, 
                                    const Eigen::Vector3d &Pdes_L,
                                     const Eigen::Matrix3d &Rdes_R,
                                      const Eigen::Vector3d &Pdes_R)
{
const pinocchio::SE3 oMdesL(Rdes_L, Pdes_L);
    const pinocchio::SE3 oMdesR(Rdes_R, Pdes_R);
    // This model is legs-only (12 DOF: left leg 0-5, right leg 6-11).
    Eigen::VectorXd qIk = Eigen::VectorXd::Zero(model_fixedbase_.nv); // initial guess
    // Eigen::VectorXd pertInitial = Eigen::VectorXd::Constant(model_biped_fixed.nv, 0.05);
    // qIk += pertInitial;
    qIk[3] = 0.1; // left knee
    qIk[9] = 0.1; // right knee

    qIk[0] = -0.08; // left hip pitch
    qIk[6] = -0.08; // right hip pitch

    qIk[5] = -0.05; // left hip roll
    qIk[11] = -0.08; // right hip roll

    const double eps = 1e-4;
    const int IT_MAX = 100;
    const double DT = 7e-1;
    const double damp = 5e-3;
    Eigen::MatrixXd JL(6, model_fixedbase_.nv);
    Eigen::MatrixXd JR(6, model_fixedbase_.nv);
    Eigen::MatrixXd JCompact(12, model_fixedbase_.nv);
    JL.setZero();
    JR.setZero();
    JCompact.setZero();

    bool success = false;
    Eigen::Matrix<double, 6, 1> errL, errR;
    Eigen::Matrix<double, 12, 1> errCompact;
    Eigen::VectorXd v(model_fixedbase_.nv);

    pinocchio::JointIndex J_Idx_l, J_Idx_r;
    J_Idx_l = left_leg_joint_ids_.back() - 1;
    J_Idx_r = right_leg_joint_ids_.back() - 1;
    int itr_count{0};
    for (itr_count = 0;; itr_count++)
    {
        pinocchio::forwardKinematics(model_fixedbase_, data_fixedbase_, qIk);
        const pinocchio::SE3 iMdL = data_fixedbase_.oMi[J_Idx_l].actInv(oMdesL);
        const pinocchio::SE3 iMdR = data_fixedbase_.oMi[J_Idx_r].actInv(oMdesR);
        errL = pinocchio::log6(iMdL).toVector(); // in joint frame
        errR = pinocchio::log6(iMdR).toVector(); // in joint frame
        errCompact.block<6, 1>(0, 0) = errL;
        errCompact.block<6, 1>(6, 0) = errR;
        if (errCompact.norm() < eps)
        {
            success = true;
            
            break;
        }
        if (itr_count >= IT_MAX)
        {
            success = false;
            break;
        }

        pinocchio::computeJointJacobian(model_fixedbase_, data_fixedbase_, qIk, J_Idx_l, JL); // JL in joint frame
        pinocchio::computeJointJacobian(model_fixedbase_, data_fixedbase_, qIk, J_Idx_r, JR); // JR in joint frame
        Eigen::MatrixXd W;
        W = Eigen::MatrixXd::Identity(model_fixedbase_.nv, model_fixedbase_.nv); // weighted matrix
        // (OpenLoong's full-humanoid version zeroes out a waist-joint block here
        // to discourage the IK solver from using it; this model is legs-only,
        // there's no waist DOF to discourage.)
        pinocchio::Data::Matrix6 JlogL;
        pinocchio::Data::Matrix6 JlogR;
        pinocchio::Jlog6(iMdL.inverse(), JlogL);
        pinocchio::Jlog6(iMdR.inverse(), JlogR);
        JL = -JlogL * JL;
        JR = -JlogR * JR;
        JCompact.block(0, 0, 6, model_fixedbase_.nv) = JL;
        JCompact.block(6, 0, 6, model_fixedbase_.nv) = JR;
        // pinocchio::Data::Matrix6 JJt;
        Eigen::Matrix<double, 12, 12> JJt;
        JJt.noalias() = JCompact * W * JCompact.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -W * JCompact.transpose() * JJt.ldlt().solve(errCompact);
        qIk = pinocchio::integrate(model_fixedbase_, qIk, v * DT);
    }

    IkRes res;
    res.err = errCompact;
    res.itr = itr_count;

    if (success)
    {
        res.status = 0;
    }
    else
    {
        res.status = -1;
    }
    res.jointPosRes = qIk;
    return res;   
}

VectorXd RobotWrapper::computeInitial_Stand(const double base_height)
{
    const double foot_height = 0.07; // distance between the foot ankel joint and the bottom
    const double  xv_des = 0.7;  // desired velocity in x direction

    const double width_hips = 0.334;

    Vector3d fe_l_pos_L_des = {0.0, width_hips / 2, -base_height};  // desired left feet pos
    Vector3d fe_r_pos_L_des = {0.0, -width_hips / 2, -base_height}; // desired right feet pos

    Vector3d fe_l_eul_L_des = {0.0, 0.0, 0.0};
    Vector3d fe_r_eul_L_des = {0.0, 0.0, 0.0};
    Matrix3d fe_l_rot_des = eul2Rot(fe_l_eul_L_des(0), fe_l_eul_L_des(1), fe_l_eul_L_des(2));
    Matrix3d fe_r_rot_des = eul2Rot(fe_r_eul_L_des(0), fe_r_eul_L_des(1), fe_r_eul_L_des(2));
    
    auto resLeg = this->computeInK_Leg(fe_l_rot_des, fe_l_pos_L_des, fe_r_rot_des, fe_r_pos_L_des);

    return resLeg.jointPosRes;
}