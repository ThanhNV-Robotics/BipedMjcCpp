#include "robot_wrapper.h"
#include "data_type.h"
#include "iostream"
#include <cstddef>
#include <string>
#include <random>
#include <vector>

#include <pinocchio/parsers/urdf.hpp>
#include <mujoco/mujoco.h>

#include "matplotlibcpp.h"

namespace pin = pinocchio;
using namespace std;

namespace plt = matplotlibcpp;

double generate_random (double min, double max)
{
    // generate a random legs' joint position and velocity within the limit
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> value_distance(min, max);
    double output = value_distance(gen);

    return output;
}

const string URDF_PATH = "models/urdf/biped_robot_12dof.urdf";
const string MJCF_PATH = "models/mjcf/biped_robot_floatingbase_12dof.xml";
const string LEFT_FOOT_BODY_NAME = "left_ankle_pitch_link";
const string FREE_JOINT_NAME = "floating_base_joint";
const double PI = 3.14;

// Actuated joint order matching RobotWrapper's internal qj/dqj layout, i.e.
// Pinocchio's own actuated-joint order (see printModelInfo(): left leg then
// right leg, each hip_pitch -> hip_roll -> hip_yaw -> knee_pitch ->
// ankle_roll -> ankle_pitch). The MJCF happens to declare joints in the same
// order, but we look each one up by name to not rely on that.
const std::vector<string> ACTUATED_JOINT_NAMES = {
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_pitch_joint", "left_ankle_roll_joint", "left_ankle_pitch_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_pitch_joint", "right_ankle_roll_joint", "right_ankle_pitch_joint",
};

int main ()
{
    cout<<"Test robot wrapper"<<endl;

    RobotWrapper robot_wrapper(URDF_PATH);
    robot_wrapper.printModelInfo();

    // Test Jacobian given a random joint configuration
    const Vector3d base_pos_min = {-0.3,-0.3,0.8};
    const Vector3d base_pos_max = {0.3,0.3,1};
    const Vector3d base_rpy_min = {-PI/6,-PI/6,-PI/6};
    const Vector3d base_rpy_max = {PI/6,PI/6,PI/6};

    const Vector3d base_linear_vel_max = {2,2,2};
    const Vector3d base_angular_vel_max = {10,10,10};

    const double joint_pos_min = -PI/2;
    const double joint_pos_max = PI/2;
    const double joint_vel_max = 5;

    const int numOfpoints = 50; //test for 50 random variable

    // Load a MuJoCo model of the same robot, used purely for a forward-
    // kinematics cross-check of the left foot velocity computed via the
    // Pinocchio Jacobian above -- no stepping/dynamics involved.
    char mj_load_error[1000] = "";
    mjModel* mj_model = mj_loadXML(MJCF_PATH.c_str(), nullptr, mj_load_error, sizeof(mj_load_error));
    if (!mj_model)
    {
        std::cerr << "Failed to load MJCF: " << MJCF_PATH << "\nMuJoCo error: " << mj_load_error << std::endl;
        return 1;
    }
    mjData* mj_data = mj_makeData(mj_model);

    const int left_foot_body_id = mj_name2id(mj_model, mjOBJ_BODY, LEFT_FOOT_BODY_NAME.c_str());
    const int free_joint_id = mj_name2id(mj_model, mjOBJ_JOINT, FREE_JOINT_NAME.c_str());
    const int free_qpos_adr = mj_model->jnt_qposadr[free_joint_id];
    const int free_qvel_adr = mj_model->jnt_dofadr[free_joint_id];

    std::vector<int> act_qpos_adr(robot_wrapper.model_na_), act_qvel_adr(robot_wrapper.model_na_);
    for (int j = 0; j < robot_wrapper.model_na_; j++)
    {
        const int jid = mj_name2id(mj_model, mjOBJ_JOINT, ACTUATED_JOINT_NAMES[j].c_str());
        act_qpos_adr[j] = mj_model->jnt_qposadr[jid];
        act_qvel_adr[j] = mj_model->jnt_dofadr[jid];
    }

    // sample data, collected for the Pinocchio-vs-MuJoCo comparison plot
    std::vector<double> samplePlot;
    std::vector<Vector3d> vFootPinPlot, vFootMjPlot;
    std::vector<Vector3d> wFootPinPlot, wFootMjPlot;

    for (int i = 1; i < numOfpoints; i++)
    {
        // generate a random base position
        double qb_x = generate_random(base_pos_min[0], base_pos_max[0]);
        double qb_y = generate_random(base_pos_min[1], base_pos_max[1]);
        double qb_z = generate_random(base_pos_min[2], base_pos_max[2]);

        // generate a random base RPY/orientation

        double qb_R = generate_random(base_rpy_min[0], base_rpy_max[0]);
        double qb_P = generate_random(base_rpy_min[1], base_rpy_max[1]);
        double qb_Y = generate_random(base_rpy_min[2], base_rpy_max[2]);

        // generate a randome joint position
        VectorXd qj = VectorXd::Zero(robot_wrapper.model_na_);
        for (int j = 0; j < robot_wrapper.model_na_; j++)
        {
            qj[j] = generate_random(joint_pos_min, joint_pos_max);
        }

        // generate random base linear velocity
        Vector3d base_lin_vel;
        base_lin_vel[0] = generate_random(-base_linear_vel_max[0], base_linear_vel_max[0]);
        base_lin_vel[1] = generate_random(-base_linear_vel_max[1], base_linear_vel_max[1]);
        base_lin_vel[2] = generate_random(-base_linear_vel_max[2], base_linear_vel_max[2]);

        // generate randome base angular velocity
        Vector3d base_ang_vel;
        base_ang_vel[0] = generate_random(-base_angular_vel_max[0], base_angular_vel_max[0]);
        base_ang_vel[1] = generate_random(-base_angular_vel_max[1], base_angular_vel_max[1]);
        base_ang_vel[2] = generate_random(-base_angular_vel_max[2], base_angular_vel_max[2]);

        // generate randome joint linear velocity
        VectorXd dqj = VectorXd::Zero(robot_wrapper.model_na_);
        for (int j = 0; j < robot_wrapper.model_na_; j++)
        {
            dqj[j] = generate_random(-joint_vel_max, joint_vel_max);
        }

        // stack to robot configuration vector and velocity vector
        Quat base_quat = AngleAxis(qb_Y, Vector3d::UnitZ())
                        * AngleAxis(qb_P, Vector3d::UnitY())
                        * AngleAxis(qb_R, Vector3d::UnitX());

        VectorXd q = VectorXd::Zero(robot_wrapper.model_nq_);
        q.segment<3>(0) = Vector3d(qb_x, qb_y, qb_z); // base position
        q.segment<4>(3) = base_quat.coeffs(); // base quaternion
        q.segment(7, robot_wrapper.model_na_) = qj; // joint position

        VectorXd v = VectorXd::Zero(robot_wrapper.model_nv_);
        v.segment<3>(0) = base_lin_vel;
        v.segment<3>(3) = base_ang_vel;
        v.segment(6, robot_wrapper.model_na_) = dqj;

        std::cout << "\n[sample " << i << "] q = " << q.transpose() << '\n';
        std::cout << "[sample " << i << "] v = " << v.transpose() << '\n';

        // pack to RobotConfiguration q_config
        RobotConfiguration q_config(robot_wrapper.model_na_);
        q_config.pos_b_W = Vector3d(qb_x, qb_y, qb_z);
        q_config.quat_b_W = base_quat;
        q_config.qj = qj;

        // Compute left foot Jacobian using computeLeftFeetJointJacobianGlobal from robot_wrapper
        Jacobian6 J_lf = robot_wrapper.computeLeftFeetJointJacobianGlobal(q_config);

        // std::cout << "Left foot Jacobian J_lf (6 x " << J_lf.cols() << "):\n" << J_lf << '\n';

        // Compute left foot linear and angular velocity from Jacobian matrix.
        // J_lf is in LOCAL_WORLD_ALIGNED frame: rows 0-2 are the linear
        // velocity of the joint origin expressed along world-aligned axes,
        // rows 3-5 are the angular velocity -- both already in world-aligned
        // axes, so J_lf * v directly gives quantities comparable to MuJoCo.
        Vector3d v_foot_pin = (J_lf * v).head<3>();
        Vector3d w_foot_pin = (J_lf * v).tail<3>();

        // Compute using mj forward
        mj_data->qpos[free_qpos_adr + 0] = qb_x;
        mj_data->qpos[free_qpos_adr + 1] = qb_y;
        mj_data->qpos[free_qpos_adr + 2] = qb_z;
        // MuJoCo free-joint quat order is (w,x,y,z); Pinocchio's coeffs() is (x,y,z,w)
        mj_data->qpos[free_qpos_adr + 3] = base_quat.w();
        mj_data->qpos[free_qpos_adr + 4] = base_quat.x();
        mj_data->qpos[free_qpos_adr + 5] = base_quat.y();
        mj_data->qpos[free_qpos_adr + 6] = base_quat.z();
        for (int j = 0; j < robot_wrapper.model_na_; j++)
        {
            mj_data->qpos[act_qpos_adr[j]] = qj[j];
        }

        // MuJoCo free-joint qvel is [0:3] = linear velocity of the body
        // origin in the WORLD frame, [3:6] = angular velocity in the
        // body-LOCAL frame. Pinocchio's v has both in the base-LOCAL frame,
        // so only the linear part needs rotating into world frame.
        Vector3d base_lin_vel_world = base_quat * base_lin_vel;
        mj_data->qvel[free_qvel_adr + 0] = base_lin_vel_world.x();
        mj_data->qvel[free_qvel_adr + 1] = base_lin_vel_world.y();
        mj_data->qvel[free_qvel_adr + 2] = base_lin_vel_world.z();
        mj_data->qvel[free_qvel_adr + 3] = base_ang_vel.x();
        mj_data->qvel[free_qvel_adr + 4] = base_ang_vel.y();
        mj_data->qvel[free_qvel_adr + 5] = base_ang_vel.z();
        for (int j = 0; j < robot_wrapper.model_na_; j++)
        {
            mj_data->qvel[act_qvel_adr[j]] = dqj[j];
        }

        mj_forward(mj_model, mj_data);

        // Positional and rotational Jacobians (3 x nv each, row-major) of the
        // left foot body's frame origin, evaluated at the current mj_data
        // state. Both map qvel to world-frame Cartesian quantities, same as
        // Pinocchio's LOCAL_WORLD_ALIGNED convention.
        std::vector<double> jacp_buf(3 * mj_model->nv);
        std::vector<double> jacr_buf(3 * mj_model->nv);
        mj_jacBody(mj_model, mj_data, jacp_buf.data(), jacr_buf.data(), left_foot_body_id);
        Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>> jacp(jacp_buf.data(), 3, mj_model->nv);
        Eigen::Map<const Eigen::Matrix<double, 3, Eigen::Dynamic, Eigen::RowMajor>> jacr(jacr_buf.data(), 3, mj_model->nv);
        Eigen::Map<const VectorXd> qvel_full(mj_data->qvel, mj_model->nv);
        Vector3d v_foot_mj = jacp * qvel_full;
        Vector3d w_foot_mj = jacr * qvel_full;

        std::cout << "[sample " << i << "] v_foot_pin = " << v_foot_pin.transpose()
                   << " | v_foot_mj = " << v_foot_mj.transpose()
                   << " | v_diff = " << (v_foot_pin - v_foot_mj).transpose() << '\n';
        std::cout << "[sample " << i << "] w_foot_pin = " << w_foot_pin.transpose()
                   << " | w_foot_mj = " << w_foot_mj.transpose()
                   << " | w_diff = " << (w_foot_pin - w_foot_mj).transpose() << '\n';

        samplePlot.push_back(i);
        vFootPinPlot.push_back(v_foot_pin);
        vFootMjPlot.push_back(v_foot_mj);
        wFootPinPlot.push_back(w_foot_pin);
        wFootMjPlot.push_back(w_foot_mj);
    }

    mj_deleteData(mj_data);
    mj_deleteModel(mj_model);

    // Plot Pinocchio vs MuJoCo left foot linear and angular velocity, one figure per axis
    auto component = [](const std::vector<Vector3d>& v, int idx) {
        std::vector<double> out(v.size());
        for (size_t k = 0; k < v.size(); k++) out[k] = v[k](idx);
        return out;
    };

    const char* axis_name[3] = {"x", "y", "z"};
    for (int axis = 0; axis < 3; axis++)
    {
        plt::figure();
        plt::named_plot("pinocchio", samplePlot, component(vFootPinPlot, axis), "--");
        plt::named_plot("mujoco", samplePlot, component(vFootMjPlot, axis), ":");
        plt::xlabel("sample");
        plt::ylabel(std::string("left foot v_") + axis_name[axis] + " [m/s]");
        plt::legend();
    }
    for (int axis = 0; axis < 3; axis++)
    {
        plt::figure();
        plt::named_plot("pinocchio", samplePlot, component(wFootPinPlot, axis), "--");
        plt::named_plot("mujoco", samplePlot, component(wFootMjPlot, axis), ":");
        plt::xlabel("sample");
        plt::ylabel(std::string("left foot w_") + axis_name[axis] + " [rad/s]");
        plt::legend();
    }
    plt::show();

    return 0;
}