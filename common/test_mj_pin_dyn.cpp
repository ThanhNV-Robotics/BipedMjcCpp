#include "test_mj_pin_dyn.h"
#include "data_type.h"
#include <iostream>

//constructor
TestDyn::TestDyn(const std::string& urdf_path, const std::string& xml_path)
    : robot_wrapper_(urdf_path)
    , mj_model_(nullptr)
    , mj_data_(nullptr)
{
    // Init mj_model and mj_data
    loadXml(xml_path, mj_model_, mj_data_);
}

bool TestDyn::testPinMjcJacobians (const std::string link_name)
{
    const int n_samples = 50;
    double mse = 0;
    for (int i = 0; i < n_samples; i++)
    {
        // Generate a random configuration
        RobotConfiguration q = generateRandomConfiguration(this->robot_wrapper_);
        // Generate a random robot spatial velocity    
        RobotSpatialVelocity v = generateRandomRobotSpatialVelocity(this->robot_wrapper_);
        // Update robot wrapper state
        this->robot_wrapper_.updateRobotState(q, v);
        // Compute jacobians
        robot_wrapper_.computeKin();
        Jacobian6 Jac;
        if (link_name == "left_ankle_pitch_link")
        {
             Jac = this->robot_wrapper_.J_Lfeet_W;
        }
        if (link_name == "right_ankle_pitch_link")
        {
             Jac = this->robot_wrapper_.J_Rfeet_W;
        }

        // pinocchio computation
        Vector6d v_pin_W = Vector6d::Zero();
        v_pin_W = Jac * v.getFlatVelocityVector(); // accept global velocity input

        // mujoco computation
        Vector6d v_mjc_W = Vector6d::Zero();
        v_mjc_W = mjcComputeBodyLinkVel(q, v, link_name);

        //computed mean squared error between 2 vector
        mse += (v_pin_W - v_mjc_W).squaredNorm();
    }
    const int N = (n_samples * 6); // mean over all samples and all 6 components
    double rmse = std::sqrt(mse / N);
    std::cout << "RMSE between Pinocchio and MuJoCo velocity ("
              << link_name << "): " << mse << std::endl;

    const double mse_tol = 1e-6;
    
    return rmse < mse_tol;
}

bool TestDyn::testPinMjcPosition(const std::string link_name)
{
    const int n_samples = 50;
    double mse = 0;
    for (int i = 0; i < n_samples; i++)
    {
        // Generate a random configuration
        RobotConfiguration q = generateRandomConfiguration(this->robot_wrapper_);
        // Generate a random robot spatial velocity    
        RobotSpatialVelocity v = generateRandomRobotSpatialVelocity(this->robot_wrapper_);
        // Update robot wrapper state
        this->robot_wrapper_.updateRobotState(q, v);
        // Compute jacobians
        robot_wrapper_.computeKin();

        Vector3d pos_feet_W;
        if (link_name == "left_ankle_pitch_link")
        {
             pos_feet_W = this->robot_wrapper_.pos_L_feet_W;
        }
        if (link_name == "right_ankle_pitch_link")
        {
             pos_feet_W = this->robot_wrapper_.pos_R_feet_W;
        }

        // mujoco computation
        Vector3d pos_feet_mjc_W = mjcComputeBodyLinkPos(q, v, link_name);

        // computed mean squared error between 2 vector
        mse += (pos_feet_W - pos_feet_mjc_W).squaredNorm();
    }
    const int N = (n_samples * 3); // mean over all samples and all 3 position components
    double rmse = std::sqrt(mse / N);
    std::cout << "RMSE between Pinocchio and MuJoCo position ("
              << link_name << "): " << rmse << std::endl;

    const double mse_tol = 1e-6;

    return rmse < mse_tol;
}

bool TestDyn::testPinMjcOrientation (const std::string link_name)
{
    const int n_samples = 50;
    double mse = 0;
    for (int i = 0; i < n_samples; i++)
    {
        // Generate a random configuration
        RobotConfiguration q = generateRandomConfiguration(this->robot_wrapper_);
        // Generate a random robot spatial velocity    
        RobotSpatialVelocity v = generateRandomRobotSpatialVelocity(this->robot_wrapper_);
        // Update robot wrapper state
        this->robot_wrapper_.updateRobotState(q, v);
        // Compute jacobians
        robot_wrapper_.computeKin();
        
        Matrix3d rot_feet_W;
        if (link_name == "left_ankle_pitch_link")
        {
             rot_feet_W = this->robot_wrapper_.rot_L_feet_W;
        }
        if (link_name == "right_ankle_pitch_link")
        {
             rot_feet_W = this->robot_wrapper_.rot_R_feet_W;
        }

        // mujoco computation
        Matrix3d rot_feet_mjc_W = mjcComputeBodyLinkRot(q, v, link_name);

        // computed mean squared error between 2 vector
        mse += (rot_feet_W - rot_feet_mjc_W).norm();
    }
    const int N = (n_samples * 9); // mean over all samples and all 3 position components
    double rmse = std::sqrt(mse / N);
    std::cout << "RMSE between Pinocchio and MuJoCo Rotation ("
              << link_name << "): " << rmse << std::endl;

    const double mse_tol = 1e-4;

    return rmse < mse_tol;    
}

Vector6d TestDyn::mjcComputeBodyLinkVel (RobotConfiguration q, RobotSpatialVelocity v, const std::string link_name)
{
    // update mujoco state first
    updateMujocoState(q, v);
    const int link_id = mj_name2id(mj_model_, mjOBJ_BODY, link_name.c_str());
    std::vector<double> jacp_buf(3 * mj_model_->nv); // position
    std::vector<double> jacr_buf(3 * mj_model_->nv); // rotation
    mj_jacBody(mj_model_, mj_data_, jacp_buf.data(), jacr_buf.data(), link_id);
    Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>> jacp(jacp_buf.data(), 3, mj_model_->nv);
    Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>> jacr(jacr_buf.data(), 3, mj_model_->nv);

    // Transform MuJoCo Jacobians to accept global angular velocity in v.getFlatVelocityVector()
    Matrix3d base_rot = q.quat_b_W.toRotationMatrix();
    MatrixXd Mpj_mjc = MatrixXd::Identity(mj_model_->nv, mj_model_->nv);
    Mpj_mjc.block<3, 3>(3, 3) = base_rot.transpose(); // MuJoCo qvel[3:6] is in local frame

    Vector3d v_link_mj = (jacp * Mpj_mjc) * v.getFlatVelocityVector(); // linear velocity in World frame
    Vector3d w_link_mj = (jacr * Mpj_mjc) * v.getFlatVelocityVector(); // angular velocity in World frame
    
    Vector6d v_link;
    v_link << v_link_mj, w_link_mj;
    return v_link;
}

Vector3d TestDyn::mjcComputeBodyLinkPos (RobotConfiguration q, RobotSpatialVelocity v, const std::string link_name)
{
    // update mujoco state first
    updateMujocoState(q, v);
    const int link_id = mj_name2id(mj_model_, mjOBJ_BODY, link_name.c_str());
    // mj_data_->xpos is the body frame origin position in World frame,
    // the same point Pinocchio's oMi[joint_id].translation() refers to.
    return Vector3d(mj_data_->xpos[3 * link_id + 0],
                     mj_data_->xpos[3 * link_id + 1],
                     mj_data_->xpos[3 * link_id + 2]);
}

Matrix3d TestDyn::mjcComputeBodyLinkRot (RobotConfiguration q, RobotSpatialVelocity v, const std::string link_name)
{
    updateMujocoState(q, v);
    const int link_id = mj_name2id(mj_model_, mjOBJ_BODY, link_name.c_str());
    const double* rot_ptr = mj_data_->xmat + 9 * link_id;

    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rot_mj(rot_ptr);
    return rot_mj;
}

void TestDyn::updateMujocoState (RobotConfiguration q, RobotSpatialVelocity v)
{
    // Compute velocity using mj_forward
    const std::string FREE_JOINT_NAME = "floating_base_joint";
    const int free_joint_id = mj_name2id(mj_model_, mjOBJ_JOINT, FREE_JOINT_NAME.c_str());
    const int free_qpos_adr = mj_model_->jnt_qposadr[free_joint_id];
    const int free_qvel_adr = mj_model_->jnt_dofadr[free_joint_id]; 
    
    // assign qpos
    mj_data_->qpos[free_qpos_adr + 0] = q.pos_b_W[0];
    mj_data_->qpos[free_qpos_adr + 1] = q.pos_b_W[1];
    mj_data_->qpos[free_qpos_adr + 2] = q.pos_b_W[2];
    // MuJoCo free-joint quat order is (w,x,y,z); Pinocchio's coeffs() is (x,y,z,w)
    mj_data_->qpos[free_qpos_adr + 3] = q.quat_b_W.w();
    mj_data_->qpos[free_qpos_adr + 4] = q.quat_b_W.x();
    mj_data_->qpos[free_qpos_adr + 5] = q.quat_b_W.y();
    mj_data_->qpos[free_qpos_adr + 6] = q.quat_b_W.z();

    for (int j = 0; j < robot_wrapper_.model_na_; j++)
    {
        mj_data_->qpos[7+j] = q.qj[j];
    }
    
    // assign qvel
    // For linear velocity of floating base, mujoco refer to World frame
    mj_data_->qvel[free_qvel_adr + 0] = v.vb_W.x(); 
    mj_data_->qvel[free_qvel_adr + 1] = v.vb_W.y();
    mj_data_->qvel[free_qvel_adr + 2] = v.vb_W.z();

    // For angular velocity, mujoco refer to LOCAL body frame
    // So we have to transform global angular to LOCAL frame
    Vector3d base_ang_vel = q.quat_b_W.inverse() * v.wb_W; // transform to LOCAL frame

    mj_data_->qvel[free_qvel_adr + 3] = base_ang_vel.x();
    mj_data_->qvel[free_qvel_adr + 4] = base_ang_vel.y();
    mj_data_->qvel[free_qvel_adr + 5] = base_ang_vel.z();
    
    // assign joint velocity
    for (int j = 0; j < robot_wrapper_.model_na_; j++)
    {
        // offset by 6 bcz of the floating base velocity
        mj_data_->qvel[6+j] = v.dq_j[j];
    }
    mj_forward(mj_model_, mj_data_);        
}

void TestDyn::loadXml(const std::string& xml_in, mjModel*& model, mjData*& data)
{
    // Delete old data/model
    if (data != nullptr)
    {
        mj_deleteData(data);
        data = nullptr;
    }

    if (model != nullptr)
    {
        mj_deleteModel(model);
        model = nullptr;
    }
    // Error buffer
    char error[1000] = {0};
    // Load new model
    model = mj_loadXML(xml_in.c_str(), nullptr, error, sizeof(error));
    // Check loading result
    if (model == nullptr)
    {
        std::cerr << "Failed to load MuJoCo XML:\n" << error << std::endl;
    }
    // Create corresponding data
    data = mj_makeData(model);

    if (data == nullptr)
    {
        std::cerr << "Failed to create mjData." << std::endl;
        mj_deleteModel(model);
        model = nullptr;
    }
}

double TestDyn::generate_random (double min, double max)
{
    // generate a random legs' joint position and velocity within the limit
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> value_distance(min, max);
    double output = value_distance(gen);

    return output;
}

RobotConfiguration TestDyn::generateRandomConfiguration (const RobotWrapper& robot_wrapper)
{
    const double PI = 3.14159;
    const Vector3d base_pos_min = {-0.3,-0.3,0.8};
    const Vector3d base_pos_max = {0.3,0.3,1};
    const Vector3d base_rpy_min = {-PI/6,-PI/6,-PI/6};
    const Vector3d base_rpy_max = {PI/6,PI/6,PI/6};

    const Vector3d base_linear_vel_max = {2,2,2};
    const Vector3d base_angular_vel_max = {10,10,10};

    const int nv = robot_wrapper.model_nv_;

    // generate a random base position
    double qb_x = generate_random(base_pos_min[0], base_pos_max[0]);
    double qb_y = generate_random(base_pos_min[1], base_pos_max[1]);
    double qb_z = generate_random(base_pos_min[2], base_pos_max[2]);

    // generate a random base RPY/orientation
    double qb_R = generate_random(base_rpy_min[0], base_rpy_max[0]);
    double qb_P = generate_random(base_rpy_min[1], base_rpy_max[1]);
    double qb_Y = generate_random(base_rpy_min[2], base_rpy_max[2]);
    Quat base_quat = AngleAxis(qb_Y, Vector3d::UnitZ())
                    * AngleAxis(qb_P, Vector3d::UnitY())
                    * AngleAxis(qb_R, Vector3d::UnitX()); // convert to quaternion

    // generate a randome joint position
    VectorXd qj = VectorXd::Zero(robot_wrapper.model_na_);
    for (int j = 0; j < robot_wrapper.model_na_; j++)
    {
        const double joint_pos_min = robot_wrapper.min_joint_pos_[j];
        const double joint_pos_max = robot_wrapper.max_joint_pos_[j];
        qj[j] = generate_random(joint_pos_min, joint_pos_max);
    }    

    // pack to RobotConfiguration
    RobotConfiguration q_config(robot_wrapper.model_na_);
    q_config.pos_b_W = Vector3d(qb_x, qb_y, qb_z);
    q_config.quat_b_W = base_quat;
    q_config.qj = qj;

    return q_config;
}

RobotSpatialVelocity TestDyn::generateRandomRobotSpatialVelocity (const RobotWrapper& robot_wrapper)
{
    // generate random base linear velocity
    Vector3d base_lin_vel;
    base_lin_vel[0] = generate_random(-2, 2);
    base_lin_vel[1] = generate_random(-2, 2);
    base_lin_vel[2] = generate_random(-2, 2);

    // generate randome base angular velocity
    Vector3d base_ang_vel;
    base_ang_vel[0] = generate_random(-10, 10);
    base_ang_vel[1] = generate_random(-10, 10);
    base_ang_vel[2] = generate_random(-10, 10);

    // generate random joint velocity
    VectorXd dqj = VectorXd::Zero(robot_wrapper.model_na_);
    for (int i = 0; i < robot_wrapper.model_na_; i++)
    {
        dqj[i] = generate_random(-robot_wrapper.joint_vel_limit_[i], robot_wrapper.joint_vel_limit_[i]);
    }

    // stack to RobotSpatialVelocity
    RobotSpatialVelocity v(robot_wrapper.model_na_);
    v.vb_W = base_lin_vel;
    v.wb_W = base_ang_vel;
    v.dq_j = dqj;
    return v;
}